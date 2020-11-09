#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import Temperature
from django_interface.msg import SilviaStatus
from django_interface.srv import SilviaStatusRequest, SilviaStatusRequestResponse

class StatusServer:
    """
    Store machine status through subscription to status topic
    Respond to service requests for status
    Required as Arduino service calls do not work in ROS Melodic
    """
    def __init__(self):
        self.status_msg = SilviaStatus(mode=0, brew=False)
        # Subscribers
        self.status_subscriber = rospy.Subscriber("status", SilviaStatus, self.status_callback, queue_size=5)
        # Service server
        self.status_server = rospy.Service('status_request', SilviaStatusRequest, self.handle_status_request)

    def status_callback(self, msg):
        self.status_msg = msg
   
    def handle_status_request(self, req):
        return SilviaStatusRequestResponse(status=self.status_msg)


if __name__ == '__main__':
    rospy.init_node("status_server_node", anonymous=True)
    server = StatusServer()

    rospy.spin()
 