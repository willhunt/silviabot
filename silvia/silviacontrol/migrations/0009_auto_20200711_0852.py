# Generated by Django 3.0.6 on 2020-07-11 07:52

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('silviacontrol', '0008_auto_20200710_1020'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='settingsmodel',
            name='V',
        ),
        migrations.AddField(
            model_name='settingsmodel',
            name='m',
            field=models.FloatField(default=20),
        ),
    ]