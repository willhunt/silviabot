from django.db.models.signals import pre_save, post_delete, post_save
from django.dispatch import receiver
from django.conf import settings as django_settings
from .tasks import async_ros_set_settings
from .models import ScheduleModel, ResponseModel, SettingsModel, SessionModel
from django_celery_beat.models import CrontabSchedule, PeriodicTask, IntervalSchedule

@receiver(pre_save, sender=ScheduleModel)
def save_schedule(sender, instance, raw, using, update_fields, **kwargs):
    """
    When creating schedule model also create linked django_celery_beat schedule entries
    """
    dow_crontype = ScheduleModel.convert_dow_to_crontype(instance.days)
    # ON SCHEDULE
    if instance.schedule_on is None:
        crontab_on = CrontabSchedule.objects.create(
            minute=instance.t_on.minute,
            hour=instance.t_on.hour,
            day_of_week=dow_crontype,
            day_of_month='*',
            month_of_year='*',
            timezone=django_settings.TIME_ZONE
        )
        schedule_on = PeriodicTask.objects.create(
            crontab=crontab_on,
            name="on:{0} {1}".format(instance.id, instance.name),
            # task='silviacontrol.tasks.async_machine_on',
            task='silviacontrol.tasks.async_ros_update',
            args="[{}, false]".format(django_settings.MODE_PID),  # json format required
            enabled=instance.active
        )
        instance.schedule_on = schedule_on
    else:
        instance.schedule_on.name = "on:{0} {1}".format(instance.id, instance.name)
        instance.schedule_on.crontab.minute = instance.t_on.minute
        instance.schedule_on.crontab.hour = instance.t_on.hour
        instance.schedule_on.crontab.day_of_week = dow_crontype
        instance.schedule_on.crontab.save()
        instance.schedule_on.enabled = instance.active
        instance.schedule_on.save()
    # OFF SCHEDULE
    if instance.schedule_off is None:
        crontab_off = CrontabSchedule.objects.create(
            minute=instance.t_off.minute,
            hour=instance.t_off.hour,
            day_of_week=dow_crontype,
            day_of_month='*',
            month_of_year='*',
            timezone=django_settings.TIME_ZONE
        )
        schedule_off = PeriodicTask.objects.create(
            crontab=crontab_off,
            name="off:{0} {1}".format(instance.id, instance.name),
            # task='silviacontrol.tasks.async_machine_off',
            task='silviacontrol.tasks.async_ros_status',
            args="[{}, false]".format(django_settings.MODE_PID),  # json format required
            enabled=instance.active
        )
        instance.schedule_off = schedule_off
    else:
        instance.schedule_off.name = "off:{0} {1}".format(instance.id, instance.name)
        instance.schedule_off.crontab.minute = instance.t_off.minute
        instance.schedule_off.crontab.hour = instance.t_off.hour
        instance.schedule_off.crontab.day_of_week = dow_crontype
        instance.schedule_off.crontab.save()
        instance.schedule_off.enabled = instance.active
        instance.schedule_off.save()

@receiver(post_delete, sender=ScheduleModel)
def delete_schedule(sender, instance, **kwargs):
    """
    When deleting schedule model also delete linked django_celery_beat schedule entries
    """
    try:
        instance.schedule_on.delete()
    except (AssertionError, AttributeError) as e:
        print('No on schedule')
    try:
        instance.schedule_off.delete()
    except (AssertionError, AttributeError) as e:
        print('No off schedule')
    try:
        instance.schedule_on.crontab.delete()
    except (AssertionError, AttributeError) as e:
        print('No Crontab on')
    try:
        instance.schedule_off.crontab.delete()
    except (AssertionError, AttributeError) as e:
        print('No Crontab off')

@receiver(post_save, sender=SettingsModel)
def save_settings(sender, instance, raw, using, update_fields, **kwargs):
    """
    When updating settings publish
    """
    async_ros_set_settings.delay()

@receiver(post_save, sender=ResponseModel)
def save_response(sender, instance, raw, using, update_fields, **kwargs):
    """
    When saving response model check if sessions needs updating
    """
    last_response = ResponseModel.objects.order_by('-t')[0]
    # If machine turned on
    if instance.mode != django_settings.MODE_OFF and last_response.mode == django_settings.MODE_OFF:
        session = SessionModel()
        session.save()
    # If machine turned off
    if instance.mode == django_settings.MODE_OFF and last_response.mode != django_settings.MODE_OFF:
        # Get current session
        try:
            # There should only be one session but end all in case there was a previous issue.
            active_sessions = SessionModel.objects.filter(active=True).order_by('-id')
            for active_session in active_sessions:
                active_session.set_end_time()
                active_session.save()
        except IndexError:
            print("No active session")
