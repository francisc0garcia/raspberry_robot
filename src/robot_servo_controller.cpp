
// ROS dependencies
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <unistd.h>
#include <pigpio.h>

int max_pwm = 255;
double scaling_pwm = 1;
int pwm_R = 0;
int pwm_L = 0;
int rate = 100;

std::string topic_name = "/cmd_vel";

#define M1_PWM 13
#define M2_PWM 6

// Robot specific
double radius = 0.16;
double L_p = 0.6655;

double right = 0;
double left = 0;

// Reset motors: put zero all outputs
void resetPWM(){
    //gpioWrite(M1_PWM, PI_LOW);
    //gpioWrite(M2_PWM, PI_LOW);

    gpioWrite(M1_PWM, PI_HIGH);
    gpioWrite(M2_PWM, PI_HIGH);

    // + 100 us from RPI3 delay
    usleep(1400);

    gpioWrite(M1_PWM, PI_LOW);
    gpioWrite(M2_PWM, PI_LOW);

    usleep(20000);
}

void move_motor_left(double left){
    int direction = 0;
    if (left > 0){
        direction = -1;
    } else if (left < 0){
        direction = 1;
    }

    switch(direction) {
        case 1: // 1.3 ms ON - 20 ms OFF
            gpioWrite(M1_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1200);
            gpioWrite(M1_PWM, PI_LOW);
            usleep(20000);
            break;
        case -1: // 1.7 ms ON - 20 ms OFF
            gpioWrite(M1_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1600);
            gpioWrite(M1_PWM, PI_LOW);
            usleep(20000);
            break;
        case 0: // 1.5 ms ON - 20 ms OFF
            // if motor doesn't stop, please calibrate it using physical pot behind motor
            gpioWrite(M1_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1400);
            gpioWrite(M1_PWM, PI_LOW);
            usleep(20000);
            break;
    }
}

void move_motor_right(double right){
    int direction = 0;
    if (right > 0){
        direction = 1;
    } else if (right < 0){
        direction = -1;
    }

    switch(direction) {
        case 1: // 1.3 ms ON - 20 ms OFF
            gpioWrite(M2_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1200);
            gpioWrite(M2_PWM, PI_LOW);
            usleep(20000);
            break;
        case -1: // 1.7 ms ON - 20 ms OFF
            gpioWrite(M2_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1600);
            gpioWrite(M2_PWM, PI_LOW);
            usleep(20000);
            break;
        case 0: // 1.5 ms ON - 20 ms OFF
            // if motor doesn't stop, please calibrate it using physical pot behind motor
            gpioWrite(M2_PWM, PI_HIGH);
            // + 100 us from RPI3 delay
            usleep(1400);
            gpioWrite(M2_PWM, PI_LOW);
            usleep(20000);
            break;
    }
}

void TwistCallback(geometry_msgs::Twist::ConstPtr Twist_temp)
{
    double dx = Twist_temp->linear.x; // vel->linear.x;
    double dr = Twist_temp->angular.z; // vel->angular.z;

    right = ((100/15)/(2*radius) ) * (dx + L_p * dr);
    left = ((100/15)/(2*radius) ) * (dx - L_p * dr);
}

int main(int argc, char **argv)
{
    // Define config for GPIO frequency
    //gpioCfgClock(4, 0, 0);

    while(gpioInitialise()<0){
        ROS_INFO("Initializing gpio...");
        sleep(1);
    }

    // Define pins for PWM and digital outputs
    //gpioSetPWMfrequency(M1_PWM, 25000);
    //gpioSetPWMfrequency(M2_PWM, 25000);

    gpioSetMode(M1_PWM, PI_OUTPUT);
    gpioSetMode(M2_PWM, PI_OUTPUT);

    resetPWM();

    ros::init(argc, argv, "robot_twist_subscriber");
    ros::NodeHandle n;
    ros::NodeHandle pnode("~");

    //Read parameters from launch file
    pnode.getParam("max_pwm", max_pwm);
    pnode.getParam("scaling_pwm", scaling_pwm);
    pnode.getParam("topic_name", topic_name);
    pnode.getParam("rate", rate);

    ros::Subscriber sub = n.subscribe(topic_name, 1, TwistCallback);
    ros::Rate r(rate);

    while ( ros::ok() )
    {
        move_motor_right(right);
        move_motor_left(left);

        ros::spinOnce();
    }

    resetPWM();

    ROS_INFO("Turning off robot_twist_subscriber");

    return 0;
}
