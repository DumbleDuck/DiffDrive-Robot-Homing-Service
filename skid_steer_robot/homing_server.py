import rclpy                                #ROS2 python client
from rclpy.node import Node                 #Class for creating node
from geometry_msgs.msg import Twist         #Msg definition for sending to /cmd_vel
from nav_msgs.msg import Odometry           #Msg definition for parsing from /odom
import math                                                     
import time
from custom_interfaces.srv import Homing    #Custom service interface package

class HomingServiceServer(Node):            #Server node class

    def __init__(self):
        #Initialize node 
        super().__init__('homing_server')                  
        self.get_logger().info('Homing Server node started')

        #Service init named homing, with service interface Homing.srv
        self.homing_service = self.create_service(Homing, 'homing', self.homing_callback)  
        self.get_logger().info('Homing service is ready.')

        #Publisher to /cmd_vel for velcity control
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #Subscriber to /odom for reading odometry data
        self.current_pose = None
        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)

        #Homing parameters 
        self.target_x = 0.0             #Default homing: X
        self.target_y = 0.05            #Default homing: Y
        self.target_yaw = 0.0           #Default homing: Yaw
        self.homing_active= False       #Flag to control homing

        self.linear_speed = 1.2         #Max lin speed while homing
        self.angular_speed = 0.5        #Max ang speed while homing
        self.position_tolerance = 0.3   #Acceptable error wrt position
        self.yaw_tolerance = 0.5        #Acceptable error wrt yaw 
        self.update_rate_hz = 50        #Control loop frequency

        #Call homing() at 50Hz with homing() being callback
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.homing)

    #Subscriber callback function
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose          #Store current robot pose

    #Service callback function    
    def homing_callback(self, request, response):  
        self.target_x = request.target_x           #Store target x,y,yaw in class attributes for further acccess
        self.target_y = request.target_y
        self.target_yaw = request.target_yaw
        
        #Sending service response in case /odom is not being published
        if self.current_pose is None:               
            response.success = False
            response.message = "Odometry not published. Is simulation running?"
            self.homing_active = False
            return response

        #Response if /odom is being published, start homing
        self.homing_active = True  
        response.success = True
        response.message = "Homing started..."
        self.get_logger().info(response.message)
        return response

    #Quaternion to euler conversion (https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code_2)
    def quaternion_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    #Stop robot motion
    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    #Get current pose only if pose is published
    def get_robot_pose(self):
        if self.current_pose is None:
            return None, None, None
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        return current_x, current_y, current_yaw

    #Main homing function triggered by the service
    def homing(self):  
        self.get_logger().info("Homing tick...")
        
        #Don't home unless flag is true
        if not self.homing_active:
            return

        self.get_logger().info(f'Target: x={self.target_x:.2f}, y={self.target_y:.2f}, yaw={self.target_yaw:.2f} rad')
        twist_msg = Twist()  #Twist class object definition

        #Get current pose. If no pose received, send info on CLI.
        current_x, current_y, current_yaw = self.get_robot_pose()
        if current_x is None:
            self.get_logger().info("No pose received. Is simulation running?")
            self.stop_robot()
            self.homing_active= False
            return

        print(f"Current X: {current_x}, Current Y: {current_y}, Current Yaw: {current_yaw}")

        #X, Y distance of target from current pose, also minimum distance 
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance_to_target_pos = math.sqrt(dx**2 + dy**2)

        #Stop homing if the distance is within specified tolerance
        if abs(distance_to_target_pos) < self.position_tolerance:
            self.get_logger().info("Reached. Stopping...")
            self.stop_robot()
            self.homing_active= False
            return

        #Set linear velocity proportional to distance. Cap it to a maximum value
        twist_msg.linear.x = (dx/abs(dx))* min(self.linear_speed, distance_to_target_pos * 1.5)

        #Publish the velocity
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f"Publishing Twist: linear.x={twist_msg.linear.x:.2f},")

#ROS2 main function
def main(args=None):
    rclpy.init(args=args)
    homing_service_server = HomingServiceServer()  #Create HomingServiceServer class object
    try:
        rclpy.spin(homing_service_server)          #Keep the node running
    except KeyboardInterrupt:
        pass                                   
    finally:
        homing_service_server.destroy_node()       #Destroy node on exit
        if rclpy.ok():
            rclpy.shutdown()                       #Shutdown ROS services if still running

if __name__ == '__main__':
    main()