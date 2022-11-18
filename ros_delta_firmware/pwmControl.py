import rospy
from std_msgs.msg import Float32, Empty

rospy.init_node('pwm_control', anonymous=True)

class pwmControl:
    def __init__(self):
        self.pwm_pub = rospy.Publisher('/motor_pwm', Float32, queue_size=1)
        self.stop = rospy.Publisher('/stop_motor', Empty, queue_size=1)

    def set_pwm(self, pwm):
        self.pwm_pub.publish(pwm)

    def stop_motor(self):
        self.stop.publish()

pwm_control = pwmControl()


if __name__ == '__main__':
    while True:
        comand = input("Enter PWM: ")
        if comand == 'stop':
            pwm_control.stop_motor()
            break   
        pwm_control.set_pwm(float(comand))
