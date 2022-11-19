from flask import Flask, request, json
import rospy
from rospy_message_converter import json_message_converter
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Vector3
from threading import Thread

global json_str

json_str = ""

app = Flask(__name__)

Thread(target=lambda: rospy.init_node('dc_motor_test_app_node', disable_signals=True, anonymous=True)).start()


def getMotorData(data):
    json_str = json_message_converter.convert_ros_message_to_json(data)
    return json_str

rospy.Subscriber('/monitoring', Vector3, getMotorData, queue_size=10)
pub_angle = rospy.Publisher('/set_angle', String, queue_size=10)
pub_stop = rospy.Publisher('/stop', Empty, queue_size=10)

@app.route('/motor/info')
def motor_info():
    rospy.wait_for_message('/monitoring', Vector3)
    return json_str

@app.route('/motor/setAngle' , methods=['POST'])
def set_angle():
    angle = request.form['angle']
    pub_angle.publish(angle)
    return angle

@app.route('/motor/setSpeed' , methods=['POST'])
def set_speed():
    return "null"
    
@app.route('/motor/stop' , methods=['POST'])
def stop():
    pub_stop.publish()

if __name__ == "__main__":
    app.run()