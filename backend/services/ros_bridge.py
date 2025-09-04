# services/ros_bridge.py
import rospy
from std_msgs.msg import String

# Only initialize once
if not rospy.core.is_initialized():
    rospy.init_node("flask_bridge", anonymous=True)

task_pub = rospy.Publisher("/robot/task", String, queue_size=10)

def send_task_to_ros(task_id: str):
    rospy.loginfo(f"Flask sending task: {task_id}")
    task_pub.publish(task_id)
