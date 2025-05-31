import rclpy
from rclpy.time import Time
from webots_ros2_core import WebotsNode
from webots_ros2_driver.utils import euler_to_quarternion
from math import sin,cos,tan,pi
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Point,Pose,Quaternion, Twist, Vector3
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
#from tf_transformations import quaternion_from_euler

class SlamEnable(WebotsNode):

    def __init__(self,args):
        super(). __init__('Slam Enable'.args)

    #enabling 3 sensors
        self.service_node_vel_timestep = 32
    #Sensor Section
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback
        )

        self.right_sensor = self.robot.getDistanceSensor('distance_sensor_right')
        self.right_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_right = self.create_publisher(Float64, 'right_IR',1)

        self.mid_sensor = self.robot.getDistanceSensor('distance_sensor_mid')
        self.mid_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_mid = self.create_publisher(Float64, 'mid_IR',1)

        self.left_sensor = self.robot.getDistanceSensor('distance_sensor_left')
        self.left_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_left = self.create_publisher(Float64, 'left_IR',1)

        #Front Wheels
        self.left_motor_front = self.robot.getMotor('left_front_wheel')
        self.left_motor_front.setPosition(float('Inf'))
        self.left_motor_front.setVelocity(0)

        self.right_motor_front = self.robot.getMotor('right_front_wheel')
        self.right_motor_front.setPosition(float('Inf'))
        self.right_motor_front.setVelocity(0)
        
        #Rear Wheels
        self.left_motor_rear = self.robot.getMotor('left_rear_wheel')
        self.left_motor_rear.setPosition(float('Inf'))
        self.left_motor_rear.setVelocity(0)

        self.right_motor_rear = self.robot.getMotor('right_front_wheel')
        self.right_motor_rear.setPosition(float('Inf'))
        self.right_motor_rear.setVelocity(0)
        #Position Sensors
        self.left_wheel_sensor = self.robot.getPositionSensor('left front position')
        self.right_wheel_sensor = self.robot.getPositionSensor('right_front_position')
        self.left_wheel_sensor.enable(self.timestep)
        self.right_wheel_sensor.enable(self.timestep)
               
        #Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmdVel_callback, 1)

        #Create Lidar Subscriber
        self.lidar_sensor = self.robot.getLidar("Lidar Sensor")
        self.lidar_sensor.enable(self.service_node_vel_timestep)
        self.laser_publisher = self.create_publisher(LaserScan, '/scan',1)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step = 0.032
        self.left_omega = 0.0 
        self.right_omega = 0.0
        self.prev_angle = 0.0
        self.prev_left_wheel_ticks = 0.0
        self.prev_right_wheel_ticks = 0.0
        self.last_time = 0.0
        self.wheel_gap = 0.12
        self.wheel_radius = 0.04
        self.front_back = 0.1

        self.odom_pub = self.create_publisher(Odometry, 'Odom',1)
        self.odom_timer = self.create_timer(self.time_step,self.odom_callback)
        self.get_logger().info('sensor enabled')
    def sensor_callback(self):
        #Publish distance sensor values
        msg_right = Float64()
        msg_right.data = self.right_sensor.getValue()
        self.sensor_publisher_right.publish(msg_right)

        msg_mid = Float64()
        msg_mid.data = self.mid_sensor.getValue()
        self.sensor_publisher_mid.publish(msg_mid)

        msg_left = Float64()
        msg_left.data = self.left_sensor.getValue()
        self.sensor_publisher_left.publish(msg_left)

        self.laser_pub()
    def laser_pub(self):
        msg_lidar = LaserScan()
        msg_lidar.header.frame_id = 'base_link'
        stamp = Time(seconds=self.robot.getTime().to_msg())
        msg_lidar.header.stamp = stamp
        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 2 * 22/7
        msg_lidar.angle_increment = (0.25 * 22)/(100 *7)
        msg_lidar.range_min = 0.12
        msg_lidar.range_max = 2.0
        msg_lidar.scan_time = 0.032
        msg_lidar.ranges = self.lidar_sensor.getRangeImage()
        
        self.laser_publishe.publish(msg_lidar)

    def cmdvel_callback(self,msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        left_speed = { (2.0* msg.linear.x - msg.angular.z * self.wheel_gap) / (2.0* self.wheel_radius)}
        right_speed = { (2.0* msg.linear.x + msg.angular.z * self.wheel_gap) / (2.0* self.wheel_radius)}
        left_speed = min(self.motor_max_speed,max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,max(-self.motor_max_speed, right_speed))

        self.left_omega = left_speed / (self.wheel_radius)
        self.right_omega = right_speed / (self.wheel_radius)
        self.left_motor_front.setVelocity(left_speed)
        self.right_motor_front.setVelocity(right_speed)
        self.left_motor_rear.setVelocity(left_speed)
        self.right_motor_rear.setVelocity(right_speed)

    def odom_callback(self):
        self.publish_odom()
    def publish_odom(self):
        stamp = Time(seconds= self.robot.getTime()).to_msg()
        self.odom_broadcaster = TransformBroadcaster(self)
        time_diff_s = self.robot.getTime() - self.last_time
        right_wheel_ticks = self.right_wheel_sensor.getValue()
        left_wheel_ticks = self.left_wheel_sensor.getValue()
        if time_diff_s == 0.0 :
           return
        #velocity calculation
        v_left_rad = (left_wheel_ticks - self.prev_left_wheel_ticks)/time_diff_s
        v_right_rad = (right_wheel_ticks - self.prev_right_wheel_ticks)/ time_diff_s
        v_left = v_left_rad * self.wheel_radius
        v_right = v_right_rad * self.wheel_radius
        v = (v_left + v_right)/2
        omega = (v_right - v_left) / 2* self.wheel_gap 

        self.prev_angle = self.th
        self.prev_left_wheel_ticks = left_wheel_ticks
        self.prev_right_wheel_ticks = right_wheel_ticks
        self.last_time = self.robot.getTime()
        #quarternion from yaw
        odom_quarternion = euler_to_quarternion(0.0,0.0,self.th)
        
        
        #publish tf
        odom_transform =  TransformStamped()
        odom_transform.header.stamp = stamp
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_link'
        odom.transform.transform.rotation = odom_quat
        odom_transform.transform.rotation.x = odom_quarternion[0]
        odom_transform.transform.rotation.y = odom_quarternion[1]
        odom_transform.transform.rotation.z = odom_quarternion[2]
        odom_transform.transform.rotation.w = odom_quarternion[3]
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0

        self.odom_broadcaster.sendTransform(odom_transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame.id = 'odom'
        odom.child_frame_id = 'base_link'
        #position set
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = odom_quat

        #velocity set
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        #publish the message
        self.odom_pub.publish(odom)

    def main(args=None):
        rclpy.init(args=args)
        robot_object = SlamEnable(args=args)
        rclpy.spin(robot_object)
        robot_object.destroy_node()
        rclpy.shutdown()    

if __name__== '__main__':
   main()