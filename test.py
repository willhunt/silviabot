import roslibpy
import time

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

status_publisher = roslibpy.Topic(client, '/status_change', 'django_interface/SilviaStatus', latch=False)
status_publisher.

msg_content = {"mode": 2, "brew": False}
status_publisher.publish(roslibpy.Message(msg_content))

time.sleep(2)

status_publisher.unadvertise()

client.terminate()