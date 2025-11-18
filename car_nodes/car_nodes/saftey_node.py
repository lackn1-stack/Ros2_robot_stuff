#!/usr/bin/env python3
# Nate Lack MTP - 10.9.2025
# Emergency Brake Node
import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Ebreak(Node):
    def __init__(self):
        super().__init__('saftey_node')

        #THE CAR KNOWS WHERE IT IS AND WHERE IT ISNT
        #BY SUBTRACTING FROM WHERE THE CAR IS NOW
        #AND WHERE THE CAR WAS BEFORE
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.scan_callback,
            10
        )




            #code that checks what the lidar is doing
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )



        self.ackerman_pub = self.create_publisher(
        AckermannDriveStamped,
        '/drive',
        10
        )
        #keeps the current velociyt of the car



        self.current_velocity = 0.0
        # Ajust these ==============
        self.TTC_THRESHOLD = 1.0 #seconds
        self.MIN_DISTANCE = 0.1 #meters
        # ==========================



    def scan_callback(self, msg: Odometry):
        self.current_velocity = msg.twist.twist.linear.x

#lidar callback
    def lidar_callback(self, msg: LaserScan):
        if self.current_velocity <= 0.0: return
        min_ttc = float('inf')
        dangerous_angle = None
        dangerous_distance = None


        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance): continue #skips if NaN or inf
            if distance < max(msg.range_min, self.MIN_DISTANCE): continue #skips if too close
            if distance > msg.range_max: continue #skips if too far

            angle = msg.angle_min + i * msg.angle_increment #angle of the ray
            range_rate = self.current_velocity * math.cos(angle) #how fast the object said ray detect is approching
            if range_rate > 0:
                ttc = distance / range_rate #time to collision
                if ttc < min_ttc:
                    min_ttc = ttc #update min ttc 
                    dangerous_angle = angle #for dipslay
                    dangerous_distance = distance #for display
        if min_ttc < self.TTC_THRESHOLD:
            self.get_logger().warn(f'Dangerous object detected! Angle: {math.degrees(dangerous_angle):.2f} deg, Distance: {dangerous_distance:.2f} m, TTC: {min_ttc:.2f} s')
            self.emergency_brake()
            
        







    #self.emergency_brake()


#Ebreak Code
    def emergency_brake(self):
        breake_msg = AckermannDriveStamped()
        breake_msg.drive.speed = 0.0
        self.ackerman_pub.publish(breake_msg)
        self.get_logger().info('Emergency Brake Activated!')
    
        



def main(args=None):
    rclpy.init(args=args)
    node = Ebreak()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()