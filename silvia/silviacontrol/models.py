from django.db import models
from django.contrib.postgres.fields import ArrayField
from django.utils import timezone
from datetime import datetime, time
from django_celery_beat.models import CrontabSchedule, PeriodicTask
from random_word import RandomWords

def get_default_pressure_profile():
    return [9, 9]
def get_default_time_profile():
    return [0, 40]

class SettingsModel(models.Model):
    """
    PID settings for the temperature control system
    """
    # Heater
    temperature_setpoint = models.FloatField(default=95)  # Temperature setpoint [degC]
    heater_kp = models.FloatField(default=0)  # Proportional gain
    heater_kd = models.FloatField(default=0)  # Derivative gain
    heater_ki = models.FloatField(default=0)  # Integral gain
    heater_kp_mode = models.IntegerField(default=1)  # Proportional or error=1, on measurement=0
    # Scale
    mass_setpoint = models.FloatField(default=20)  # Extraction mass [g]
    # Sampling times
    t_update = models.IntegerField(default=5)  # Time delay between client side updates [s]
    t_sample = models.IntegerField(default=5)  # Time delay between server side sampling [s]
    # Cleaning
    t_clean_on = models.IntegerField(default=10)  # Time to turn pump on for during cleaning cycle [s]
    t_clean_off = models.IntegerField(default=50)  # Time to turn pump off for during cleaning cycle [s]
    n_clean_cycles = models.IntegerField(default=5)  # Number of on/off cycles [-]
    # Pressure profile
    pressure_setpoints = ArrayField(models.FloatField(default=get_default_pressure_profile))  # Pressure profile
    pressure_t_setpoints = ArrayField(models.FloatField(default=get_default_time_profile))  # Time profile (for pressure profile)
    pump_kp = models.FloatField(default=0)  # Proportional gain
    pump_kd = models.FloatField(default=0)  # Derivative gain
    pump_ki = models.FloatField(default=0)  # Integral gain

    def __repr__(self):
        repr_str = ('Settings %d' % self.id)
        return repr_str


class SessionModel(models.Model):
    """
    Reference session for each machine on/off cycle
    Holds all temperature records for session as database relationship
    """
    t_start = models.DateTimeField(default=timezone.now)  # Session start time
    t_end = models.DateTimeField(blank=True, null=True)  # Session end time
    active = models.BooleanField(default=True)  # Current session flag

    def set_end_time(self, t_end=None, active=False):
        """
        ARGUMENTS:
        t_end (DateTime): Session end time
        active (Bool): Session active flag
        """
        if t_end is None:
            self.t_end = timezone.now()
        else:
            self.t_end = t_end
        self.active = active

    @property
    def start_date(self):
        return timezone.localtime(self.t_start).strftime("%Y-%m-%d") 

    @property
    def start_time(self):
        return timezone.localtime(self.t_start).strftime("%H:%M")

    def __repr__(self):
        repr_str = ("Session %d - %s"  % (self.id, self.t_start.strftime("%Y-%m-%d %H:%M")))
        return repr_str


class ResponseModel(models.Model):
    """
    Machine temperature over time
    PID response over time
    """
    t = models.DateTimeField(auto_now=True)
    mode = models.IntegerField(default=0)
    brewing = models.BooleanField(default=False)
    low_water = models.BooleanField(default=False)
    
    temperature = models.FloatField()
    temperature_setpoint = models.FloatField(blank=True, null=True)
    heater_duty = models.FloatField(default=0)
    heater_kp = models.FloatField(blank=True, null=True)
    heater_ki = models.FloatField(blank=True, null=True)   
    heater_kd = models.FloatField(blank=True, null=True)

    mass = models.FloatField(default=0, null=True)
    mass_setpoint = models.FloatField(default=0, null=True)

    pressure = models.FloatField(blank=True, null=True)
    pressure_setpoint = models.FloatField(blank=True, null=True)
    pump_duty = models.FloatField(default=0)
    pump_kp = models.FloatField(blank=True, null=True)
    pump_ki = models.FloatField(blank=True, null=True)   
    pump_kd = models.FloatField(blank=True, null=True)
    
    @property
    def response_date(self):
        return self.t.date()

    @property
    def response_time(self):
        return self.t.strftime("%H:%M:%S")

    def __repr__(self):
        repr_str = ("T=%d @ %s" % (self.temperature, self.t.strftime("%Y-%m-%d %H:%M:%S")))
        return repr_str


class ScheduleModel(models.Model):
    """
    Machine on/off schedule model
    """
    name = models.CharField(max_length=20, default="New Schedule") #('Schedule %s' % RandomWords().get_random_word()))  # Name of schedule (e.g. "Morning")
    days = models.CharField(max_length=7, default='0000000')  # Day schedule - string of 0 or 1's to indicate if active on weekday starting with Sun=0
    t_on = models.TimeField(default=time(hour=0, minute=0))
    t_off = models.TimeField(default=time(hour=0, minute=0))
    # Schedule tasks. These are not master (see above fields) as they share data. SIngle source of truth are data members above.
    schedule_on = models.ForeignKey(PeriodicTask, on_delete=models.CASCADE, related_name='schedule_on', null=True, blank=True)  # Time to turn machine on
    schedule_off = models.ForeignKey(PeriodicTask, on_delete=models.CASCADE, related_name='schedule_off', null=True, blank=True)  # Time to turn machine off
    active = models.BooleanField(default=False)  # Schedule active [True] or not [False]

    def get_weekdays(self):
        day_names = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat']
        schedule_days = []
        for active, name in zip(self.days, day_names):
            if active is '1':
                schedule_days.append(name)
        return schedule_days

    @staticmethod
    def convert_dow_to_crontype(dow):
        """
        Convert database day of the week to crontab list format
        """
        dow_cron = []
        for i, d in enumerate(dow):
            if d is "1":
                dow_cron.append(i)
        return ",".join(map(str, dow_cron))  # Return as string

    @staticmethod
    def convert_dow_from_crontype(dow):
        """
        Convert crontab style day of the week to database type
        """
        dow = map(int, dow.split(","))
        dow_cron = []
        for i, d in enumerate(dow):
            if d is "1":
                dow_cron.append(i)
        return dow_cron

    @staticmethod
    def time_to_four_digit_string(t):
        hour = str(t.hour)
        if len(hour) == 1:
            hour = "0" + hour
        minute = str(t.minute)
        if len(minute) == 1:
            minute = "0" + minute
        return ("%s:%s" % (hour, minute))

    @staticmethod
    def four_digit_string_to_time(t_string):
        try:
            hour, minute = t_string.split(":")
        except ValueError:
            raise ValueError("String must be in the format '00:00'")

        return time(hour=int(hour), minute=int(minute))

    @property
    def start_time(self):
        return self.time_to_four_digit_string(self.t_on)

    @start_time.setter
    def start_time(self, value):
        # self.start_time = value
        self.t_on = self.four_digit_string_to_time(value)

    @property
    def end_time(self):
        return self.time_to_four_digit_string(self.t_off)

    @end_time.setter
    def end_time(self, value):
        # self.end_time = value
        self.t_off = self.four_digit_string_to_time(value)

    def __repr__(self):
        if self.active:
            active_text = "Active"
        else:
            active_text = "Not active"
        repr_str = ("Schedule: %s - %s (%s)"  % (self.t_on, self.t_off, active_text))
        return repr_str

