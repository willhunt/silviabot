from __future__ import absolute_import, unicode_literals
from celery import shared_task
from celery_once import QueueOnce
from .models import ResponseModel, SettingsModel
from .utils import debug_log
from django.conf import settings as django_settings
from django.utils import timezone
import struct
import roslibpy
import requests
import time

client = roslibpy.Ros(host='localhost', port=9090)
client.run()
# Status
status_publisher = roslibpy.Topic(client, '/status_change', 'django_interface/SilviaStatus', latch=False)
status_service = roslibpy.Service(client, '/status_request', '/status')
status_request = roslibpy.ServiceRequest()
# Override
heater_duty_publisher = roslibpy.Topic(client, '/heater_duty', 'std_msgs/Float64', latch=False)
pump_duty_publisher = roslibpy.Topic(client, '/pump_duty', 'std_msgs/Float64', latch=False)
# Settings
settings_publisher = roslibpy.Topic(client, '/settings', 'django_interface/SilviaSettings', latch=False)

@shared_task(queue='comms')
def async_scale_update(brew, mass_setpoint):
    """
    Args
        on [Bool]: True = start brewing, False = stop brewing
    """
    debug_log("Scale update task, input: {}".format(brew))
    if django_settings.SIMULATE_SCALE == False:
        # Reset scale
        if brew:
            try:
                debug_log("Sending to scale at url:")
                request_scale = requests.put("http://192.168.0.12/brewstart", params={"setpoint": mass_setpoint})
                debug_log(request_scale.url)  # Check URL is correct
            except requests.exceptions.RequestException as e:
                debug_log("No connection to scale")
        else:
            try:
                request_scale = requests.put("http://192.168.0.12/brewstop")
                debug_log(request_scale.text)  # Check response
            except requests.exceptions.RequestException as e:
                debug_log("No connection to scale")

@shared_task(queue='comms')
def async_ros_set_status(mode=-1, brew=False):
    """
    Send status update via ROS through celery

    Can't get roslibpy to work using async tasks/
    """
    # local_client = roslibpy.Ros(host='localhost', port=9090)
    # local_client.run()
    # local_status_publisher = roslibpy.Topic(client, '/status_change', 'django_interface/SilviaStatus', latch=True)
    # msg_content = {"mode": int(mode)}
    # if brew is not None:
    #     msg_content["brew"] = brew in ["True", "true", True]
    # status_publisher.publish(roslibpy.Message(msg_content))
    # local_status_publisher.publish(roslibpy.Message(msg_content))
    # local_status_publisher.unadvertise()
    # local_client.terminate()
    # debug_log(client.get_topics())
    status_request = requests.put("http://localhost:8080/api/v1/status/1/", data={"mode": mode, "brew": brew})
    debug_log("Status change requested via async call: Mode={}, Brew={}".format(mode, brew))
 
@shared_task(queue='celery')
def async_save_response(response_dict):
    ResponseModel.objects.create(**response_dict)

def ros_get_status():
    """
    Get status from ROS
    """
    status_response = status_service.call(status_request)
    return status_response
    
def ros_set_status(mode=-1, brew=None):
    """
    Send status update via ROS
    """
    msg_content = {"mode": int(mode)}
    msg_content["brew"] = brew in ["True", "true", True]  # Account for string or bool
    debug_log("Advertised? {}".format(status_publisher.is_advertised))
    status_publisher.publish(roslibpy.Message(msg_content))
    debug_log("Status change requested via sync call: Mode={}, Brew={}".format(mode, brew))

def ros_set_heater(duty=0):
    """
    Control heater manually
    """
    msg = roslibpy.Message({
        "data": duty
    })
    heater_duty_publisher.publish(msg)
    debug_log("Heater duty request via sync call: Duty={}".format(duty))

def ros_set_pump(duty=0):
    """
    Control pump manually
    """
    msg = roslibpy.Message({
        "data": duty
    })
    pump_duty_publisher.publish(msg)
    debug_log("Pump duty request via sync call: Duty={}".format(duty))

def ros_set_settings(settings_dict=None):
    """
    Send settings update via ROS
    """
    if settings_dict is None:
        settings_dict = SettingsModel.objects.get(pk=1).__dict__
    msg = roslibpy.Message(settings_dict)
    settings_publisher.publish(msg)
