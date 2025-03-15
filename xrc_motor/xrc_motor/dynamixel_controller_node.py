#!/usr/bin/env python3

# Copyright 2021 ROBOTIS CO., LTD.
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
#
# Modified for flexible parameter-based configuration with AX-12A defaults

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from xrc_interfaces.msg import SetPosition
from xrc_interfaces.srv import GetPosition
#from dynamixel_sdk_custom_interfaces.msg import SetPosition
#from dynamixel_sdk_custom_interfaces.srv import GetPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

class DynamixelControllerNode(Node):

    def __init__(self):
        super().__init__('dynamixel_controller')

        # Declare parameters with default values (for AX-12A)
        self.declare_parameter('motor_type', 'ax12a')  # Options: 'x_series', 'ax12a'
        self.declare_parameter('protocol_version', 1.0)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('dxl_id', 1)

        # Get parameters
        self.motor_type = self.get_parameter('motor_type').value
        self.protocol_version = self.get_parameter('protocol_version').value
        self.baudrate = self.get_parameter('baudrate').value
        self.device_name = self.get_parameter('device_name').value
        self.dxl_id = self.get_parameter('dxl_id').value

        # Configure addresses based on motor type
        if self.motor_type == 'x_series':
            # Control table address for X series
            self.ADDR_OPERATING_MODE = 11
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.ADDR_PRESENT_POSITION = 132

            # Set protocol and baudrate if not explicitly configured
            if self.protocol_version == 1.0:
                self.get_logger().warning('Protocol 1.0 selected but X series uses 2.0. Switching to 2.0.')
                self.protocol_version = 2.0

            if self.baudrate == 1000000:
                self.get_logger().info('Using 1000000 baudrate for X series. Default is 57600.')
        else:
            # Default to AX-12A addresses
            self.ADDR_CW_ANGLE_LIMIT = 6
            self.ADDR_CCW_ANGLE_LIMIT = 8
            self.ADDR_TORQUE_ENABLE = 24
            self.ADDR_GOAL_POSITION = 30
            self.ADDR_PRESENT_POSITION = 36

        # Log configuration
        self.get_logger().info(f"Motor Type: {self.motor_type}")
        self.get_logger().info(f"Protocol Version: {self.protocol_version}")
        self.get_logger().info(f"Baudrate: {self.baudrate}")
        self.get_logger().info(f"Device: {self.device_name}")
        self.get_logger().info(f"Dynamixel ID: {self.dxl_id}")

        # Initialize handlers
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        self.get_logger().info('Succeeded to open the port.')

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Succeeded to set the baudrate.')

        self.setup_dynamixel(self.dxl_id)
        qos = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            SetPosition,
            'set_position',
            self.set_position_callback,
            qos
        )

        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)

    def setup_dynamixel(self, dxl_id):
        if self.motor_type == 'x_series':
            # Set operating mode for X series
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_OPERATING_MODE, 3  # Position Control
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set Position Control Mode: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            else:
                self.get_logger().info('Succeeded to set Position Control Mode.')
        else:
            # Set position control mode by setting CW/CCW angle limits for AX-12A
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_CW_ANGLE_LIMIT, 0
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set CW Angle Limit: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            else:
                self.get_logger().info('Succeeded to set CW Angle Limit.')

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_CCW_ANGLE_LIMIT, 1023
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set CCW Angle Limit: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
            else:
                self.get_logger().info('Succeeded to set CCW Angle Limit.')

        # Enable torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, self.ADDR_TORQUE_ENABLE, 1
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Succeeded to enable torque.')

    def set_position_callback(self, msg):
        goal_position = msg.position

        # Apply position limits for AX-12A
        if self.motor_type == 'ax12a':
            goal_position = min(max(0, goal_position), 1023)
            # Write position (2 bytes for AX-12A)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, msg.id, self.ADDR_GOAL_POSITION, goal_position
            )
        else:
            # Write position (4 bytes for X series)
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, msg.id, self.ADDR_GOAL_POSITION, goal_position
            )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Set [ID: {msg.id}] [Goal Position: {goal_position}]')

    def get_position_callback(self, request, response):
        if self.motor_type == 'ax12a':
            # Read position (2 bytes for AX-12A)
            position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                self.port_handler, request.id, self.ADDR_PRESENT_POSITION
            )
        else:
            # Read position (4 bytes for X series)
            position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler, request.id, self.ADDR_PRESENT_POSITION
            )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Get [ID: {request.id}] \
                                   [Present Position: {position}]')

        response.position = position
        return response

    def __del__(self):
        try:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                self.dxl_id,
                self.ADDR_TORQUE_ENABLE,
                0  # Disable torque
            )
            self.port_handler.closePort()
            self.get_logger().info('Shutting down dynamixel_controller')
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()