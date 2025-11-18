#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import numpy as np
import math

class CenterLineFollowingController(Node):
    def __init__(self):
        super().__init__('mtp_two_code')
        
        #PID Parameters
        self.declare_parameter('kp', 1.8)
        self.declare_parameter('ki', 0.015)
        self.declare_parameter('kd', 0.6)
        self.declare_parameter('lookahead_distance', 1)  # L in meters
        self.declare_parameter('theta_deg', 50.0)  # angle between beams
        
        #get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.theta = math.radians(self.get_parameter('theta_deg').value)
        
        #HORES BLINDERS
        self.horse = 2.3
        self.leftcorrect = 0.29

        #pid varbs
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
        #is the car runn9n?
        self.is_running = True
        
        #creating publish and subscirbers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            '/drive', 
            10
        )
        
        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Center Line Following Controller Initialized')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Lookahead: {self.lookahead_distance}m, Theta: {math.degrees(self.theta)}°')
        
    def get_range(self, scan_msg, angle_deg):
        #need angle in rads
        angle_rad = math.radians(angle_deg)
        
        #find what number beam and whre it is
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        
        #starts at 1 but python stats at 0 s need to minus 1
        index = max(0, min(index, len(scan_msg.ranges) - 1))
        
        # Get range, handle inf/nan values
        range_val = scan_msg.ranges[index]
        if math.isinf(range_val) or math.isnan(range_val):
            range_val = self.horse #blinds car to not react to things too far away
        elif range_val > self.horse:
            range_val = self.horse #blinds car to not react to things too far away
            
        return range_val
    
    def calculate_alpha_and_distance(self, a, b):
 
        #caluclating alpha using invers tang
        numerator = a * math.cos(self.theta) - b
        denominator = a * math.sin(self.theta)
        
        #divide by zero check
        if abs(denominator) < 0.001:
            alpha = 0.0
        else:
            alpha = math.atan(numerator / denominator)
        
        Dt = b * math.cos(alpha)
        
        #prediciting the future distance
        Dt_plus_1 = Dt + self.lookahead_distance * math.sin(alpha)
        
        return alpha, Dt, Dt_plus_1
    
    def pid_control(self, error, dt):

        #P
        p_term = self.kp * error
        
        #I
        self.integral_error += error * dt
        # limit term so it doesnt grow forever 
        max_integral = 5.0
        self.integral_error = np.clip(self.integral_error, -max_integral, max_integral)
        i_term = self.ki * self.integral_error
        
        #D
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        
        #steering angle
        steering_angle = p_term + i_term + d_term
        
        #N-1 error to compare with last one
        self.prev_error = error
        
        #limits steering angle to +- 30
        max_steering = math.radians(30)
        steering_angle = np.clip(steering_angle, -max_steering, max_steering)
        
        return steering_angle
    
    def calculate_speed(self, steering_angle_deg):
 
        abs_angle = abs(steering_angle_deg)
        
        if abs_angle <= 10.0:
            speed = 1.5
        elif abs_angle <= 20.0:
            speed = 1.0
        else:
            speed = 0.5
            
        return speed
    
    def publish_stop(self):
 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)
        
        #pubhish stop to car
        twist_msg = Twist()  
        self.twist_pub.publish(twist_msg)
        
        self.get_logger().info('STOP command published')
    
    def scan_callback(self, scan_msg):
        if not self.is_running:
            return
            
        try:
            #left wall
            b_left = self.get_range(scan_msg, 90)   # 90° to the left
            a_left = self.get_range(scan_msg, 90 - math.degrees(self.theta))
            
            #right wall 
            b_right = self.get_range(scan_msg, -90)  # 90° to the right
            a_right = self.get_range(scan_msg, -90 + math.degrees(self.theta))
            
            # Validate measurements
            if (b_left > scan_msg.range_max - 0.1 or a_left > scan_msg.range_max - 0.1 or
                b_right > scan_msg.range_max - 0.1 or a_right > scan_msg.range_max - 0.1):
                self.get_logger().warn('LiDAR readings at max range, walls may not be detected')
                return
            
            #math distance to both wallds
            alpha_left, Dt_left, Dt_plus_1_left = self.calculate_alpha_and_distance(a_left, b_left)
            alpha_right, Dt_right, Dt_plus_1_right = self.calculate_alpha_and_distance(a_right, b_right)
            
            # Calculate error: difference between right and left distances
            # Positive error = closer to left wall, need to steer right (positive steering)
            # Negative error = closer to right wall, need to steer left (negative steering)


            if Dt_plus_1_left + Dt_plus_1_right > 3.6: 
                error = (Dt_plus_1_left - Dt_plus_1_right) + 1

            else:
                error = (Dt_plus_1_left - Dt_plus_1_right) + self.leftcorrect
            
            #calculate time step
            current_time = self.get_clock().now()
            if self.prev_time is None:
                dt = 0.05  # Assume ~30Hz if first iteration
            else:
                dt = (current_time - self.prev_time).nanoseconds / 1e9
            self.prev_time = current_time
            
            #PID STUFF
            steering_angle = self.pid_control(error, dt)
            steering_angle_deg = math.degrees(steering_angle)
            
            #steering angle based calc
            speed = self.calculate_speed(steering_angle_deg)
            
            #publish the drive command
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = current_time.to_msg()
            drive_msg.header.frame_id = 'base_link'
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = speed
            
            self.drive_pub.publish(drive_msg)
            
            #screen torubleshooting output
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0
                
            if self.log_counter % 30 == 0:
                corridor_width = Dt_plus_1_left + Dt_plus_1_right
                self.get_logger().info(
                    f'Left: {Dt_plus_1_left:.2f}m, Right: {Dt_plus_1_right:.2f}m, '
                    f'Width: {corridor_width:.2f}m, Error: {error:.2f}m, '
                    f'Steering: {steering_angle_deg:.1f}°, Speed: {speed:.1f}m/s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')
    
    def destroy_node(self):
        self.get_logger().info('Shutting down - publishing STOP command')
        self.is_running = False
        #Run 5 times to make stop (it doesnt) (i tired adding this and a twist command to publish 0 vleocity but it always keeps running after the node is shut down)
        for _ in range(5):
            self.publish_stop()
            
        self.get_logger().info('Stop commands published')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CenterLineFollowingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
