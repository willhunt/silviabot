import roslibpy
import time

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

status_publisher = roslibpy.Topic(client, '/status_change', 'django_interface/SilviaStatus', latch=True)

msg_content = {"mode": 2, "brew": False}
status_publisher.publish(roslibpy.Message(msg_content))

time.sleep(1)

status_publisher.unadvertise()

client.terminate()