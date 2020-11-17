# This will stop tasks when using Ctrl+C
trap "kill 0" EXIT

# ROS
# roslaunch rosbridge_server rosbridge_websocket.launch &
# roslaunch django_interface django_interface.launch
# sleep 2

# Redis
redis-cli flushall

# Celery
# -c is concurrency (=1)
celery -A silvia worker --queues=celery &
celery -A silvia worker -c 1 --queues=comms &
celery -A silvia beat --scheduler django_celery_beat.schedulers:DatabaseScheduler &

# Django ROS
python manage.py ros &

# Waits until process finishes (or Ctrl+c)
wait