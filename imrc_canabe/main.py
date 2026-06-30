"""ROS2 demo node for transmitting CAN frames with a CANable adapter."""

# Copyright 2026 kei
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
import logging

import rclpy
from rclpy.node import Node


@dataclass
class CanFrameRequest:
    """Parsed CAN frame request from a demo topic message."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool


class CanableSenderNode(Node):
    """Bridge ROS topics and CAN frames for transmit and receive demos."""

    def __init__(self) -> None:
        """Initialize ROS interfaces and the CAN bus connection."""
        super().__init__('canable_sender')

        self.declare_parameter('channel', '/dev/ttyACM0')
        self.declare_parameter('bitrate', 500000)
        self.declare_parameter('receive_topic', 'can_tx_demo')
        self.declare_parameter('publish_topic', 'can_rx_demo')
        self.declare_parameter('interface', 'slcan')
        self.declare_parameter('poll_period_sec', 0.01)

        self._bus = self._create_bus()
        self._command_msg_type = self._load_command_message_type()

        topic_name = self.get_parameter('receive_topic').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        poll_period_sec = (
            self.get_parameter('poll_period_sec').get_parameter_value().double_value
        )

        self._subscription = self.create_subscription(
            self._command_msg_type,
            topic_name,
            self._topic_callback,
            10,
        )
        self._publisher = self.create_publisher(
            self._command_msg_type,
            publish_topic,
            10,
        )
        self._rx_timer = self.create_timer(poll_period_sec, self._poll_can_bus)

        self.get_logger().info(
            f'Ready: tx topic="{topic_name}", rx topic="{publish_topic}".',
        )

    def _create_bus(self):
        """Create a python-can bus based on ROS parameters."""
        can = self._load_python_can_module()

        channel = self.get_parameter('channel').get_parameter_value().string_value
        bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        interface = self.get_parameter('interface').get_parameter_value().string_value

        try:
            if hasattr(can, 'Bus'):
                bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
            else:
                bus = can.interface.Bus(
                    bustype=interface,
                    channel=channel,
                    bitrate=bitrate,
                )
        except Exception as error:  # pragma: no cover - hardware dependent
            self.get_logger().error(
                f'Failed to open CAN interface "{interface}" on "{channel}": {error}',
            )
            raise

        self.get_logger().info(
            'Connected to CAN adapter: '
            f'interface={interface}, channel={channel}, bitrate={bitrate}',
        )
        return bus

    def _topic_callback(self, msg) -> None:
        """Convert a command message into a CAN frame and transmit it."""
        try:
            frame = self._parse_frame(msg)
            self._send_frame(frame)
        except ValueError as error:
            self.get_logger().warning(f'Ignoring invalid command: {error}')
        except Exception as error:  # pragma: no cover - hardware dependent
            self.get_logger().error(f'Failed to send CAN frame: {error}')

    def _parse_frame(self, msg) -> CanFrameRequest:
        """Build a classic CAN frame from the custom ROS message."""
        unit_code = self._validate_range(
            'unit_code',
            int(msg.unit_code),
            lower=0,
            upper=0x3F,
        )
        unit_index = self._validate_range(
            'unit_index',
            int(msg.unit_index),
            lower=0,
            upper=0x0F,
        )
        payload_index = self._validate_range(
            'payload_index',
            int(msg.payload_index),
            lower=0,
            upper=0x07,
        )
        payload_entry = self._validate_range(
            'payload_entry',
            int(msg.payload_entry),
            lower=0,
            upper=0x1F,
        )
        payload_body = [int(value) for value in msg.data]
        if len(payload_body) > 7:
            raise ValueError('data must contain 0 to 7 bytes')

        for index, value in enumerate(payload_body):
            self._validate_range(f'data[{index}]', value, lower=0, upper=0xFF)

        arbitration_id = (unit_code << 5) | (unit_index << 1) | 0x01
        payload_header = (payload_index << 5) | payload_entry
        data = bytes([payload_header, *payload_body])

        return CanFrameRequest(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

    def _send_frame(self, frame: CanFrameRequest) -> None:
        """Send a parsed CAN frame to the bus."""
        can = self._load_python_can_module()

        message = can.Message(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            is_extended_id=frame.is_extended_id,
        )
        self._bus.send(message)
        self.get_logger().info(
            f'Sent CAN frame id=0x{frame.arbitration_id:X} '
            f'dlc={len(frame.data)} data={frame.data.hex().upper()}',
        )

    def _poll_can_bus(self) -> None:
        """Poll the CAN interface and publish received frames as ROS messages."""
        try:
            message = self._bus.recv(timeout=0.0)
        except Exception as error:  # pragma: no cover - hardware dependent
            self.get_logger().error(f'Failed to receive CAN frame: {error}')
            return

        if message is None:
            return

        try:
            command = self._decode_frame(message)
        except ValueError as error:
            self.get_logger().warning(
                f'Ignoring received CAN frame id=0x{message.arbitration_id:X}: {error}',
            )
            return

        self._publisher.publish(command)
        rx_data = self._extract_message_data(message)
        self.get_logger().info(
            f'Received CAN frame id=0x{message.arbitration_id:X} '
            f'dlc={len(rx_data)} data={rx_data.hex().upper()}',
        )

    def _decode_frame(self, message):
        """Convert a classic CAN frame into the custom ROS command message."""
        arbitration_id = int(message.arbitration_id)
        data = list(self._extract_message_data(message))

        if message.is_extended_id:
            raise ValueError('extended CAN frames are not supported')
        if len(data) < 1:
            raise ValueError('payload header byte is missing')
        if len(data) > 8:
            raise ValueError('classic CAN payload must be 8 bytes or less')

        command = self._command_msg_type()
        command.unit_code = (arbitration_id >> 5) & 0x3F
        command.unit_index = (arbitration_id >> 1) & 0x0F
        command.payload_index = (data[0] >> 5) & 0x07
        command.payload_entry = data[0] & 0x1F
        command.data = data[1:]
        return command

    def _extract_message_data(self, message) -> bytes:
        """Return only the valid payload bytes from a received CAN message."""
        raw_data = bytes(message.data)
        if hasattr(message, 'dlc'):
            return raw_data[:int(message.dlc)]
        return raw_data

    def destroy_node(self) -> bool:
        """Close the CAN bus before shutting down the ROS node."""
        if hasattr(self, '_bus') and self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:  # pragma: no cover - hardware dependent
                pass
        return super().destroy_node()

    def _load_python_can_module(self):
        """Import python-can and validate that the expected API is available."""
        try:
            import can
        except ImportError as error:
            self.get_logger().error(
                'python-can is not installed. Install python3-can before running this node.',
            )
            raise RuntimeError('python-can is required') from error

        self._hydrate_python_can_namespace(can)

        bus_api = getattr(can, 'Bus', None)
        message_api = getattr(can, 'Message', None)
        interface_module = getattr(can, 'interface', None)
        has_bus_api = bus_api is not None or (
            interface_module is not None and hasattr(interface_module, 'Bus')
        )
        has_message_api = message_api is not None
        if not (has_bus_api and has_message_api):
            module_path = getattr(can, '__file__', 'unknown location')
            raise RuntimeError(
                'Imported "can" module is not python-can. '
                f'Loaded module: {module_path}. '
                'Uninstall the unrelated "can" package and install python-can '
                '(for example: pip uninstall can && pip install python-can), '
                'or use the ROS/apt package python3-can.'
            )

        return can

    def _hydrate_python_can_namespace(self, can_module) -> None:
        """Patch partially installed python-can namespace packages in place."""
        required_imports = {
            'interface': ('can.interface', None),
            'Message': ('can.message', 'Message'),
            'BusABC': ('can.bus', 'BusABC'),
            'BusState': ('can.bus', 'BusState'),
            'CanProtocol': ('can.bus', 'CanProtocol'),
            'BitTiming': ('can.bit_timing', 'BitTiming'),
            'BitTimingFd': ('can.bit_timing', 'BitTimingFd'),
            'VALID_INTERFACES': ('can.interfaces', 'VALID_INTERFACES'),
            'CanInitializationError': (
                'can.exceptions',
                'CanInitializationError',
            ),
            'CanInterfaceNotImplementedError': (
                'can.exceptions',
                'CanInterfaceNotImplementedError',
            ),
            'CanOperationError': ('can.exceptions', 'CanOperationError'),
            'CanTimeoutError': ('can.exceptions', 'CanTimeoutError'),
            'typechecking': ('can.typechecking', None),
        }

        for attr_name, (module_name, exported_name) in required_imports.items():
            if hasattr(can_module, attr_name):
                continue

            imported_module = import_module(module_name)
            value = (
                imported_module
                if exported_name is None
                else getattr(imported_module, exported_name)
            )
            setattr(can_module, attr_name, value)

        if not hasattr(can_module, 'Bus'):
            setattr(can_module, 'Bus', can_module.interface.Bus)
        if not hasattr(can_module, 'rc'):
            setattr(can_module, 'rc', {})
        if not hasattr(can_module, 'log'):
            setattr(can_module, 'log', logging.getLogger('can'))

    def _load_command_message_type(self):
        """Import the custom ROS message type used for CAN commands."""
        try:
            from imrc_messages.msg import EcanCommand
        except ImportError as error:
            raise RuntimeError(
                'Custom message type "imrc_messages/msg/EcanCommand" is not available. '
                'Add the .msg file to your interface package, rebuild, and source the workspace.'
            ) from error

        return EcanCommand

    def _validate_range(
        self,
        field_name: str,
        value: int,
        *,
        lower: int,
        upper: int,
    ) -> int:
        """Validate that an integer value fits into an expected bit field."""
        if lower <= value <= upper:
            return value

        raise ValueError(f'{field_name} must be in range {lower}..{upper}, got {value}')


def main(args: list[str] | None = None) -> None:
    """Run the CANable sender node."""
    rclpy.init(args=args)
    node = CanableSenderNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
