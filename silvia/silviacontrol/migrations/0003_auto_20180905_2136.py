# Generated by Django 2.1.1 on 2018-09-05 20:36

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    dependencies = [
        ('django_celery_beat', '0006_auto_20180210_1226'),
        ('silviacontrol', '0002_auto_20180904_2158'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='schedulemodel',
            name='key_task_off',
        ),
        migrations.RemoveField(
            model_name='schedulemodel',
            name='key_task_on',
        ),
        migrations.RemoveField(
            model_name='schedulemodel',
            name='t_off',
        ),
        migrations.RemoveField(
            model_name='schedulemodel',
            name='t_on',
        ),
        migrations.AddField(
            model_name='schedulemodel',
            name='schedule_off',
            field=models.ForeignKey(null=True, on_delete=django.db.models.deletion.CASCADE, related_name='schedule_off', to='django_celery_beat.PeriodicTask'),
        ),
        migrations.AddField(
            model_name='schedulemodel',
            name='schedule_on',
            field=models.ForeignKey(null=True, on_delete=django.db.models.deletion.CASCADE, related_name='schedule_on', to='django_celery_beat.PeriodicTask'),
        ),
    ]