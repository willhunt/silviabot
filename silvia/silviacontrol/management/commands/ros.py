import roslibpy
from django.core.management.base import BaseCommand, CommandError
from silviacontrol.models import ResponseModel
from silviacontrol.utils import debug_log
from silviacontrol.tasks import async_save_response
from django.conf import settings as django_settings
from datetime import datetime
from django.utils.timezone import make_aware, now

UPDATE_INTERVAL_WARM = 5
UPDATE_INTERVAL_BREW = 0.5

class RosResponseManager:
    def __init__(self):
        self.update_interval = UPDATE_INTERVAL_WARM
        self.last_update = datetime.now()
        self.status_msg = roslibpy.Message()
        self.heater_msg = roslibpy.Message()
        self.pump_msg = roslibpy.Message()
        self.water_msg = roslibpy.Message()
        self.scale_msg = roslibpy.Message()

        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        # response_listener = roslibpy.Topic(client, '/response', 'django_interface/SilviaResponse')
        # response_listener.subscribe(self.response_callback)
        self.status_listener = roslibpy.Topic(self.client, '/status', 'django_interface/SilviaStatus')
        self.status_listener.subscribe(self.status_callback)
        self.heater_listener = roslibpy.Topic(self.client, '/heater_controller', 'django_interface/SilviaController')
        self.heater_listener.subscribe(self.heater_callback)
        self.pump_listener = roslibpy.Topic(self.client, '/pump_controller', 'django_interface/SilviaController')
        self.pump_listener.subscribe(self.pump_callback)
        self.water_listener = roslibpy.Topic(self.client, '/water_level', 'django_interface/SilviaWater')
        self.water_listener.subscribe(self.water_callback)
        self.scale_listener = roslibpy.Topic(self.client, '/scale', 'django_interface/SilviaMass')
        self.scale_listener.subscribe(self.scale_callback)


    def status_callback(self, msg):
        self.status_msg = msg
        if msg["brew"]:
            self.update_interval = UPDATE_INTERVAL_BREW
        else:
            self.update_interval = UPDATE_INTERVAL_WARM

    def heater_callback(self, msg):
        self.heater_msg = msg

    def pump_callback(self, msg):
        self.pump_msg = msg

    def water_callback(self, msg):
        self.water_msg = msg

    def scale_callback(self, msg):
        self.scale_msg = msg

    def initiate_response_log(self):
        response_dict = {
            "t": now()
        }
        try:
            response_dict["mode"] = self.status_msg["mode"]
            response_dict["brew"] = self.status_msg["brew"]
        except (AttributeError, KeyError):
            pass

        try:
            t_heater = datetime.utcfromtimestamp(int(self.heater_msg["header"]["stamp"]["secs"]))
            if (datetime.utcnow() - t_heater).total_seconds() < 5:
                response_dict["temperature"] = self.heater_msg["input"]
                response_dict["temperature_setpoint"] = self.heater_msg["setpoint"]
                response_dict["heater_duty"] = self.heater_msg["output"]
                response_dict["heater_kp"] = self.heater_msg["kp"]
                response_dict["heater_ki"] = self.heater_msg["ki"]
                response_dict["heater_kd"] = self.heater_msg["kd"]
        except (AttributeError, KeyError):
            pass

        try:
            t_pump = datetime.utcfromtimestamp(int(self.pump_msg["header"]["stamp"]["secs"]))
            if (datetime.utcnow() - t_pump).total_seconds() < 5:
                response_dict["temperature"] = self.pump_msg["input"]
                response_dict["temperature_setpoint"] = self.pump_msg["setpoint"]
                response_dict["pump_duty"] = self.pump_msg["output"]
                response_dict["pump_kp"] = self.pump_msg["kp"]
                response_dict["pump_ki"] = self.pump_msg["ki"]
                response_dict["pump_kd"] = self.pump_msg["kd"]
        except (AttributeError, KeyError):
            pass
            
        try:
            t_scale = datetime.utcfromtimestamp(int(self.scale_msg["header"]["stamp"]["secs"]))
            if (datetime.utcnow() - t_scale).total_seconds() < 5:
                response_dict["mass"] = self.scale_msg["mass"]
                response_dict["mass_setpoint"] = self.scale_msg["mass_setpoint"]
        except (AttributeError, KeyError):
            pass

        try:
            t_water = datetime.utcfromtimestamp(int(self.water_msg["header"]["stamp"]["secs"]))
            if (datetime.utcnow() - t_water).total_seconds() < 5:
                response_dict["low_water"] = self.water_msg["low_water"]
        except (AttributeError, KeyError):
            pass

        async_save_response.delay(response_dict)

    def update(self):
        if (datetime.now() - self.last_update).total_seconds() >= self.update_interval:
            # Save to database via celery to avoid slowing down loggong rate (will join queue)
            self.initiate_response_log()


class Command(BaseCommand):
    help = 'Comminication with ROS'

    def handle(self, *args, **options):
        response_manager = RosResponseManager()
        try:
            while True:
                response_manager.update()
        except KeyboardInterrupt:
            response_manager.client.terminate()
