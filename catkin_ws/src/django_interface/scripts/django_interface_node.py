#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import Temperature
from django_interface.msg import (
    SilviaResponse, SilviaStatus, SilviaWater, SilviaBrewTimer, SilviaController, SilviaMass
)

class RosDjangoInterface:
    def __init__(self):
        self.response_pub_interval = 10  # Interval for publishing machine response for saving [s]
        # Message and defaults
        self.response_msg = SilviaResponse()
        self.response_msg.header.frame_id = "pi"
        self.response_msg.status.brew = False
        self.response_msg.status.mode = 0
        # Subscribers
        self.status_subscriber = rospy.Subscriber("status", SilviaStatus, self.status_callback, queue_size=5)
        self.heater_subscriber = rospy.Subscriber("heater_controller", SilviaController, self.heater_callback, queue_size=100)
        self.pump_subscriber = rospy.Subscriber("pump_controller", SilviaController, self.pump_callback, queue_size=100)
        self.water_subscriber = rospy.Subscriber("water_level", SilviaWater, self.water_callback, queue_size=10)
        self.scale_subscriber = rospy.Subscriber("scale", SilviaMass, self.scale_callback, queue_size=100)
        # Publishers
        self.response_publisher = rospy.Publisher("response", SilviaResponse, queue_size=10)
        
    def heater_callback(self, msg):
        self.response_msg.heater_control = msg

    def pump_callback(self, msg):
        self.response_msg.pump_control = msg

    def scale_callback(self, msg):
        self.response_msg.mass = msg

    def status_callback(self, msg):
        self.response_msg.status = msg

    def water_callback(self, msg):
        self.response_msg.water = msg

    def publish_response(self):
        self.response_msg.header.stamp = rospy.Time.now()
        self.response_publisher.publish(self.response_msg)


if __name__ == '__main__':
    rospy.init_node("django_interface_node", anonymous=True)
    interface = RosDjangoInterface()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        interface.publish_response()
        rate.sleep()
 
    