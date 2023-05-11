

import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import TrajectoryWaypoint
from px4_msgs.msg import VehicleOdometry



class test1(Node):

    def __init__(self):
        super().__init__('test1')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", 10)
        self.trajectory_waypoint_publisher_ = self.create_publisher(TrajectoryWaypoint,"/fmu/in/trajectory_waypont", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.vehicle_odometry_= self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_odometry", 10)
        self.offboard_setpoint_counter_ = 0

        timer_period = 0.2  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        self.publish_offboard_control_mode()
        if (self.offboard_setpoint_counter_ > 10):
            self.publish_trajectory_setpoint()


        self.offboard_setpoint_counter_ += 1

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = True
        msg.body_rate = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)


    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, -2.0]
        self.get_logger().info("stage1")
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)
    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = test1()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
