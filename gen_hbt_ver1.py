#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Float32
from datetime import datetime

class HeartbeatPublisher:
    def __init__(self):
        rospy.init_node('heartbeat_publisher', anonymous=True)
        self.pub = rospy.Publisher('HBT', String, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz

        # Constant fields
        self.message_id = "$RXHRB"
        self.team_id = "ROBOTX_NTU"  # Team ID constant

        # Initialize dynamic fields with default values
        self.latitude = 0.0
        self.longitude = 0.0
        self.ns_indicator = 'N'
        self.ew_indicator = 'W'
        self.system_mode = 2  # Default Autonomous
        self.uav_status = 1   # Default Stowed

        # Subscribers to required topics
        rospy.Subscriber('GPS', String, self.gps_callback)
        rospy.Subscriber('MODE', Int32, self.mode_callback)
        rospy.Subscriber('UAV', Int32, self.uav_callback)

    def gps_callback(self, msg):
        # Update GPS data from topic
        data = msg.data.split(',')
        self.latitude = float(data[0])
        self.ns_indicator = data[1]
        self.longitude = float(data[2])
        self.ew_indicator = data[3]

    def mode_callback(self, msg):
        # Update system mode from MODE topic
        self.system_mode = msg.data

    def uav_callback(self, msg):
        # Update UAV status from UAV topic
        self.uav_status = msg.data

    def calculate_checksum(self, message):
        # Calculate the checksum by bitwise XOR over all characters in the message
        checksum = 0
        for char in message:
            checksum ^= ord(char)
        return f"{checksum:X}"  # Return as hexadecimal string

    def create_heartbeat_message(self):
        # Get the current date and time in U.S. Eastern Standard Time (EST)
        current_time = datetime.utcnow()
        est_date = current_time.strftime("%d%m%y")
        est_time = current_time.strftime("%H%M%S")

        # Format message without checksum
        msg_without_checksum = f"{self.message_id},{est_date},{est_time},{self.latitude},{self.ns_indicator}," \
                               f"{self.longitude},{self.ew_indicator},{self.team_id},{self.system_mode},{self.uav_status}"
        
        # Calculate checksum
        checksum = self.calculate_checksum(msg_without_checksum)
        
        # Construct the complete message with checksum
        heartbeat_message = f"${msg_without_checksum}*{checksum}"
        
        return heartbeat_message

    def publish_heartbeat(self):
        while not rospy.is_shutdown():
            heartbeat_message = self.create_heartbeat_message()
            self.pub.publish(heartbeat_message)
            rospy.loginfo(f"Published heartbeat message: {heartbeat_message}")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        hb_publisher = HeartbeatPublisher()
        hb_publisher.publish_heartbeat()
    except rospy.ROSInterruptException:
        pass
