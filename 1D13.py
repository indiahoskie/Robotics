#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range  # For ultrasonic; use LaserScan for LiDAR

class StraightDrive:
    def __init__(self):
        rospy.init_node('straight_drive_stop')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/distance_sensor', Range, self.sensor_callback)
        self.safe_distance = 0.5  # meters
        self.object_detected = False
        self.rate = rospy.Rate(10)

    def sensor_callback(self, data):
        if data.range < self.safe_distance:
            self.object_detected = True

    def drive(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Forward speed

        while not rospy.is_shutdown():
            if self.object_detected:
                move_cmd.linear.x = 0.0
                rospy.loginfo("Object detected. Stopping.")
            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = StraightDrive()
        robot.drive()
    except rospy.ROSInterruptException:
        pass

