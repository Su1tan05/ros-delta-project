#include "mbed.h"
#include "Libs/Encoder/encoder.h"
#include "Libs/Dc_servo/DcServo.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>  

#include "config.h"
#define PUB_RATE 0.1

float kp = 5;
float ki = 0.5;
float kd = 0;

float setpoint1 = 0;
float setpoint2 = 0;
float setpoint3 = 0;

Ticker Motor1_info_ticker;
Encoder Mortor1_encoder{M1_ENCA_PIN, M1_ENCB_PIN, REDUCTER_RATIO, ENCODER_IMPULSES, true};
DcServo Motor1{M1_IN1_PIN, M1_IN2_PIN, M1_PWM_PIN, &Mortor1_encoder};

Ticker Motor2_info_ticker;
Encoder Mortor2_encoder{M2_ENCA_PIN, M2_ENCB_PIN, REDUCTER_RATIO, ENCODER_IMPULSES, true};
DcServo Motor2{M2_IN1_PIN, M2_IN2_PIN, M2_PWM_PIN, &Mortor1_encoder};

Ticker Motor3_info_ticker;
Encoder Mortor3_encoder{M3_ENCA_PIN, M3_ENCB_PIN, REDUCTER_RATIO, ENCODER_IMPULSES, true};
DcServo Motor3{M3_IN1_PIN, M3_IN2_PIN, M3_PWM_PIN, &Mortor1_encoder};

ros::NodeHandle nh;

geometry_msgs::Vector3 monitoring_msg_m1;
geometry_msgs::Vector3 monitoring_msg_m2;
geometry_msgs::Vector3 monitoring_msg_m3;

void initRosTopics(void);
void stopMotor(const std_msgs::Int32 &stop_msg);

void setAngleMotor1(const std_msgs::Float32 &angle_msg);
void pidTuningsMotor1(const geometry_msgs::Vector3 &pid_tunings_msg);
void getInfoMotor1(void);

void setAngleMotor2(const std_msgs::Float32 &angle_msg);
void pidTuningsMotor2(const geometry_msgs::Vector3 &pid_tunings_msg);
void getInfoMotor2(void);

void setAngleMotor3(const std_msgs::Float32 &angle_msg);
void pidTuningsMotor3(const geometry_msgs::Vector3 &pid_tunings_msg);
void getInfoMotor3(void);

ros::Subscriber<std_msgs::Int32> stop_motor_sub("stop_motor1", &stopMotor);

ros::Subscriber<std_msgs::Float32> motor1_set_angle_sub("set_angle_motor1", &setAngleMotor1);
ros::Subscriber<geometry_msgs::Vector3> motor1_pid_tunings_sub("pid_tunings_motor1", &pidTuningsMotor1);

ros::Subscriber<std_msgs::Float32> motor2_set_angle_sub("set_angle_motor2", &setAngleMotor2);
ros::Subscriber<geometry_msgs::Vector3> motor2_pid_tunings_sub("pid_tunings_motor2", &pidTuningsMotor2);

ros::Subscriber<std_msgs::Float32> motor3_set_angle_sub("set_angle_motor3", &setAngleMotor3);
ros::Subscriber<geometry_msgs::Vector3> motor3_pid_tunings_sub("pid_tunings_motor3", &pidTuningsMotor3);

ros::Publisher m1_monitoring_pub("monitoring_m1", &monitoring_msg_m1);
ros::Publisher m2_monitoring_pub("monitoring_m2", &monitoring_msg_m2);
ros::Publisher m3_monitoring_pub("monitoring_m3", &monitoring_msg_m3);

int main()
{
    nh.initNode();
    initRosTopics();

    Motor1_info_ticker.attach(getInfoMotor1, PUB_RATE);
    Motor2_info_ticker.attach(getInfoMotor2, PUB_RATE);
    Motor3_info_ticker.attach(getInfoMotor3, PUB_RATE);

    while (1)
    {
        nh.spinOnce();
    }
}

void initRosTopics()
{
    nh.subscribe(stop_motor_sub);

    nh.advertise(m1_monitoring_pub);
    nh.subscribe(motor1_set_angle_sub);
    nh.subscribe(motor1_pid_tunings_sub);

    nh.advertise(m2_monitoring_pub);
    nh.subscribe(motor2_set_angle_sub);
    nh.subscribe(motor2_pid_tunings_sub);

    nh.advertise(m3_monitoring_pub);
    nh.subscribe(motor3_set_angle_sub);
    nh.subscribe(motor3_pid_tunings_sub);
}

void stopMotor(const std_msgs::Int32 &stop_msg)
{
    switch (stop_msg.data)
    {
        case 1:
            setpoint1 = 0;
            Motor1.stop();
            break;
        case 2:
            setpoint2 = 0;
            Motor2.stop();
            break;
        case 3:
            setpoint3 = 0;
            Motor3.stop();
            break;
        default:
            break;
    }
}

void setAngleMotor1(const std_msgs::Float32 &angle_msg){
    setpoint1 = angle_msg.data;
    monitoring_msg_m1.x = setpoint1;
    Motor1.setAngle(setpoint1);
}

void setAngleMotor2(const std_msgs::Float32 &angle_msg){
    setpoint2 = angle_msg.data;
    monitoring_msg_m2.x = setpoint2;
    Motor2.setAngle(setpoint2);
}

void setAngleMotor3(const std_msgs::Float32 &angle_msg){
    setpoint3 = angle_msg.data;
    monitoring_msg_m3.x = setpoint3;
    Motor3.setAngle(setpoint3);
}

void pidTuningsMotor1(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    Motor1.setPid(kp, ki, kd);
}

void pidTuningsMotor2(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    Motor2.setPid(kp, ki, kd);
}

void pidTuningsMotor3(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    Motor3.setPid(kp, ki, kd);
}

void getInfoMotor1(){
    monitoring_msg_m1.x = setpoint1;
    monitoring_msg_m1.y = Motor1.getAngle();
    monitoring_msg_m1.z = Motor1.getPWM();
    m1_monitoring_pub.publish(&monitoring_msg_m1);
}

void getInfoMotor2(){
    monitoring_msg_m2.x = setpoint2;
    monitoring_msg_m2.y = Motor2.getAngle();
    monitoring_msg_m2.z = Motor2.getPWM();
    m2_monitoring_pub.publish(&monitoring_msg_m2);
}

void getInfoMotor3(){
    monitoring_msg_m3.x = setpoint3;
    monitoring_msg_m3.y = Motor3.getAngle();
    monitoring_msg_m3.z = Motor3.getPWM();
    m3_monitoring_pub.publish(&monitoring_msg_m3);
}