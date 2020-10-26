import roslibpy
from django.core.management.base import BaseCommand, CommandError
from silviacontrol.models import ResponseModel
from django.conf import settings as django_settings
from silviacontrol.utils import debug_log
from datetime import datetime
from django.utils.timezone import make_aware


class Command(BaseCommand):
    help = 'Comminication with ROS'
    one_message_saved = False

    def response_callback(self, msg):
        t = make_aware(datetime.utcfromtimestamp(int(msg["header"]["stamp"]["secs"])))
        debug_log("Response topic received at {}".format(t))
        if msg["mode"] != django_settings.MODE_OFF:  # Save response if machine is on
            ResponseModel.objects.create(
                t=t,
                T_boiler=msg["boiler_temperature"],
                duty=msg["duty"],
                k_p=msg["kp"],
                k_i=msg["ki"],
                k_d=msg["kd"],
                m=msg["m"],
                T_setpoint=msg["T_set"],
                low_water=msg["low_water"],
                mode=msg["mode"],
                brewing=msg["brew"]
            )

    def status_callback(self, msg):
        pass

    def handle(self, *args, **options):
        client = roslibpy.Ros(host='localhost', port=9090)
        client.run()

        response_listener = roslibpy.Topic(client, '/response', 'django_interface/SilviaResponse')
        response_listener.subscribe(self.response_callback)

        status_listener = roslibpy.Topic(client, '/status', 'django_interface/SilviaStatus')
        status_listener.subscribe(self.status_callback)

        try:
            while True:
                pass
        except KeyboardInterrupt:
            client.terminate()