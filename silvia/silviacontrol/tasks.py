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
status_publisher = roslibpy.Topic(client, '/status_change', 'django_interface/SilviaStatus', latch=True)
status_service = roslibpy.Service(client, '/status_request', '/status')
status_request = roslibpy.ServiceRequest()
# Override
heater_duty_publisher = roslibpy.Topic(client, '/heater_duty_request', 'django_interface/SilviaHeaterDuty')
# Settings
settings_publisher = roslibpy.Topic(client, '/settings', 'django_interface/SilviaStatus')

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
def async_ros_set_status(mode=-1, brew=None):
    """
    Send status update via ROS
    """
    msg_content = {"mode": mode}
    if brew is not None:
        msg_content["brew"] = brew
        status_publisher.
    status_publisher.publish(roslibpy.Message(msg_content))
    debug_log("Status change requested via async call: Mode={}, Brew={}".format(mode, brew))
    debug_log("Advertised? {}".format(status_publisher.is_advertised))
    

@shared_task(queue='comms')
def async_ros_set_settings():
    """
    Send settings update via ROS
    """
    settings = SettingsModel.objects.get(pk=1)
    msg = roslibpy.Message(settings.__dict__)
    status_publisher.publish(msg)

@shared_task(queue='comms')
def async_ros_set_heater(duty=0):
    """
    Control override/manual mode of arduino
    """
    msg = roslibpy.Message({
        "duty": duty
    })
    heater_duty_publisher.publish(msg)
    debug_log("Heater duty request via async call: Duty={}".format(duty))
 
@shared_task(queue='celery')
def async_save_response(response_dict):
    ResponseModel.objects.create(**response_dict)

def sync_ros_get_status():
    """
    Get status from ROS
    """
    status_response = status_service.call(status_request)
    return status_response
    