#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from django_interface.msg import SilviaResponse

class RosDjangoInterface:
    def __init__(self):
        self.response_pub_interval = 10  # Interval for publishing machine response for saving [s]
        self.temperature = 0

        self.temperature_subscriber = rospy.Subscriber("micro/boiler_temperature", Temperature, self.temperature_callback, queue_size=100)
        self.response_publisher = rospy.Publisher("response", SilviaResponse, queue_size=10)
        
    def temperature_callback(self, msg):
        self.temperature = msg.temperature

    def publish_response(self):
        msg = SilviaResponse()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "micro"
        msg.heater_control.input = self.temperature
        self.response_publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node("django_interface_node", anonymous=True)
    interface = RosDjangoInterface()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        interface.publish_response()
        rate.sleep()
 
    