import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
import math
import time
from custom_interfaces.srv import Homing

class HomingServiceServer(Node):

    def __init__(self):
        super().__init__('homing_server')
        self.get_logger().info('Homing Server node started')

        # Service init
        self.homing_service = self.create_service(Homing, 'homing', self.homing_callback)
        self.get_logger().info('Homing service is ready.')

        # Publisher init
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber init
        self.current_pose = None
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)

        # Homing parameters 
        self.linear_speed = 0.2         # Max lin speed while homing
        self.angular_speed = 0.5        # Max ang speed while homing
        self.position_tolerance = 0.5  # acceptable error wrt position
        self.yaw_tolerance = 0.5        # acceptable error wrt yaw 
        self.update_rate_hz = 50        # Hz for control loop
        self.rate = self.create_rate(self.update_rate_hz)


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def quaternion_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.rate.sleep()

    def get_robot_pose(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_pose is None:
            return None, None, None

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        return current_x, current_y, current_yaw

    def homing_callback(self, request, response):  #Service callback function
        target_x = request.target_x
        target_y = request.target_y
        target_yaw = request.target_yaw

        self.get_logger().info(f'Target: x={target_x:.2f}, y={target_y:.2f}, yaw={target_yaw:.2f} rad ({math.degrees(target_yaw):.1f} deg)')

        twist_msg = Twist()

        #Align with target position vector
        # while rclpy.ok():                                                   #If ROS still running
        #     current_x, current_y, current_yaw = self.get_robot_pose()
        #     if current_x is None:
        #         response.success = False
        #         response.message = "No pose during Step 1."
        #         self.get_logger().error(response.message)
        #         self.stop_robot()
        #         return response

        #     print(f"Current X: {current_x}, Current Y: {current_y}, Current Yaw: {current_yaw}")

        #     dx = target_x - current_x
        #     dy = target_y - current_y
            
        #     distance_to_target_pos = math.sqrt(dx**2 + dy**2)
        #     if distance_to_target_pos < self.position_tolerance:
        #         self.get_logger().info("Already at target position.")
        #         break 

        #     #Calculating current yaw and position vector yaw
        #     angle_to_target_pos = math.atan2(dy, dx)
        #     yaw_error = angle_to_target_pos - current_yaw
        #     yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        #     if abs(yaw_error) < self.yaw_tolerance:
        #         self.get_logger().info("Aligned to target position vector.")
        #         self.stop_robot()
        #         break

        #     twist_msg.linear.x = 0.0
        #     twist_msg.angular.z = self.angular_speed * (1 if yaw_error > 0 else -1)
        #     self.cmd_vel_publisher.publish(twist_msg)
        #     self.rate.sleep()

        # self.stop_robot()

        #Move to target position

        while True:
            current_x, current_y, current_yaw = self.get_robot_pose()
            if current_x is None:
                response.success = False
                response.message = "No pose during Step 2."
                self.get_logger().error(response.message)
                self.stop_robot()
                return response

            print(f"Current X: {current_x}, Current Y: {current_y}, Current Yaw: {current_yaw}")
            dx = target_x - current_x
            dy = target_y - current_y
            distance_to_target_pos = math.sqrt(dx**2 + dy**2)

            if abs(distance_to_target_pos) < self.position_tolerance:
                self.get_logger().info("Reached. Stopping...")
                self.stop_robot()
                break

            twist_msg.linear.x = -min(self.linear_speed, distance_to_target_pos * 1.0)
            twist_msg.linear.x = -max(twist_msg.linear.x, 0.05) 

            # # Keep re-aligning while moving
            # angle_to_target_pos = math.atan2(dy, dx)
            # yaw_error_during_move = angle_to_target_pos - current_yaw
            # yaw_error_during_move = math.atan2(math.sin(yaw_error_during_move), math.cos(yaw_error_during_move)) # Normalize

            # twist_msg.angular.z = self.angular_speed * yaw_error_during_move * 0.5 # Proportional angular correction
            # twist_msg.angular.z = max(min(twist_msg.angular.z, self.angular_speed), -self.angular_speed) # Cap angular speed

            self.cmd_vel_publisher.publish(twist_msg)
            self.rate.sleep()
        print("Loop exited phase 1")
        self.stop_robot() 

        # #Moving to target yaw set by user
        # while rclpy.ok():
        #     current_x, current_y, current_yaw = self.get_robot_pose()
        #     if current_x is None:
        #         response.success = False
        #         response.message = "Lost robot pose during Phase 3."
        #         self.get_logger().error(response.message)
        #         self.stop_robot()
        #         return response

    
        #     final_yaw_error = target_yaw - current_yaw
        #     final_yaw_error = math.atan2(math.sin(final_yaw_error), math.cos(final_yaw_error)) # Normalize

        #     if abs(final_yaw_error) < self.yaw_tolerance:
        #         self.get_logger().info("Phase 3: Final yaw alignment achieved.")
        #         self.stop_robot()
        #         break # Exit Phase 3 loop

        #     twist_msg.linear.x = 0.0
        #     twist_msg.angular.z = self.angular_speed * (1 if final_yaw_error > 0 else -1)
            
        #     self.cmd_vel_publisher.publish(twist_msg)
        #     self.rate.sleep()

        # self.stop_robot()

        response.success = True
        response.message = f"Robot successfully homed to x={target_x:.2f}, y={target_y:.2f}, yaw={target_yaw:.2f}."
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    homing_service_server = HomingServiceServer()
    try:
        rclpy.spin(homing_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        homing_service_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()