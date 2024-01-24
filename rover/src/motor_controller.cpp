#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

class Controller
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_imu;
    ros::NodeHandle nh_joint;

    ros::Publisher FL_CON_pub;
    ros::Publisher FR_CON_pub;
    ros::Publisher RL_CON_pub;
    ros::Publisher RR_CON_pub;

    ros::Publisher Motor_FL_CON_pub;
    ros::Publisher Motor_FR_CON_pub;
    ros::Publisher Motor_RL_CON_pub;
    ros::Publisher Motor_RR_CON_pub;
    ros::Publisher Motor_ML_CON_pub;
    ros::Publisher Motor_MR_CON_pub;

    ros::Subscriber sub;
    ros::Subscriber imu_sub;
    ros::Subscriber joint_sub;

    std_msgs::Float64 FL_angle;
    std_msgs::Float64 FR_angle;
    std_msgs::Float64 RL_angle;
    std_msgs::Float64 RR_angle;

    std_msgs::Float64 FL_Wheel_Velocity;
    std_msgs::Float64 FR_Wheel_Velocity;
    std_msgs::Float64 RL_Wheel_Velocity;
    std_msgs::Float64 RR_Wheel_Velocity;
    std_msgs::Float64 ML_Wheel_Velocity;
    std_msgs::Float64 MR_Wheel_Velocity;

    double fl_vel, fr_vel, ml_vel, mr_vel, rl_vel, rr_vel;
    double ang_cur[6];
    double ang_pre[6];

    geometry_msgs::Twist Linear;

    double x = 0;
    double y = 0;

    #define d1 0.177 // m 중심과 전좌 바퀴 중앙에 대한 거리 x 축
    #define d2 0.310 // m 중심과 전좌 바퀴 중앙에 대한 거리 y축 
    #define d3 0.274 // m 중심과 전후 바퀴 중앙에 대한 거리 y축
    #define d4 0.253 // m 중심과 가운데 바퀴 중앙에 대한 거리 x축

    #define ROVER_WHEEL_RADIUS 0.075
    #define ROVER_WHEELBASE 0.506   // ? 이게 뭐지
    // drive_no_load_rpm = 223.0

    double servo_theta = 0;
    double l = 0; // linear, angular에 대한 회전반경
    double theta_front_closest;
    double theta_front_farthest;

    double angular_velocity_center, vel_middle_closest, vel_corner_closest, vel_corner_farthest, vel_middle_farthest;
    double ang_vel_middle_closest, ang_vel_corner_closest, ang_vel_corner_farthest, ang_vel_middle_farthest;
    double start_time, time, pre_time;
public:

    Controller()
    {
        FL_CON_pub = nh.advertise<std_msgs::Float64>("FL_CON/command", 2);
        FR_CON_pub = nh.advertise<std_msgs::Float64>("FR_CON/command", 2);
        RL_CON_pub = nh.advertise<std_msgs::Float64>("RL_CON/command", 2);
        RR_CON_pub = nh.advertise<std_msgs::Float64>("RR_CON/command", 2);

        Motor_FL_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_fl_controller/command", 2);
        Motor_FR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_fr_controller/command", 2);
        Motor_RL_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_rl_controller/command", 2);
        Motor_RR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_rr_controller/command", 2);
        Motor_ML_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_ml_controller/command", 2);
        Motor_MR_CON_pub = nh.advertise<std_msgs::Float64>("rover_motor_mr_controller/command", 2);
  
        sub = nh.subscribe("cmd_vel", 5, &Controller::msgCallback, this);
        joint_sub = nh_joint.subscribe("joint_states", 5, &Controller::jointCallback, this);
        imu_sub = nh_imu.subscribe("imu", 5, &Controller::imuCallback, this);
    }

    // 서보모터 각도 publish
    void publishAngles() // publishAngles 함수 정의
    {
        FL_CON_pub.publish(FL_angle);
        FR_CON_pub.publish(FR_angle);
        RL_CON_pub.publish(RL_angle);
        RR_CON_pub.publish(RR_angle);
    }

    // wheel 각속도 publish
    void publishVelocity() // publishAngles 함수 정의
    {
        Motor_FL_CON_pub.publish(FL_Wheel_Velocity);
        Motor_FR_CON_pub.publish(FR_Wheel_Velocity);
        Motor_RL_CON_pub.publish(RL_Wheel_Velocity);
        Motor_RR_CON_pub.publish(RR_Wheel_Velocity);
        Motor_ML_CON_pub.publish(ML_Wheel_Velocity);
        Motor_MR_CON_pub.publish(MR_Wheel_Velocity);
    }

    // unsigned char mode (geometry_msgs::Twist::ConstPtr& msg)
    // {
    //     unsigned char mode_value; //255가지 
    //     if((msg->linear.x == 0)&&(msg->angular.z == 0))
    //     {
    //         mode_value = 1;

    //     }

    // }

    // void servo_pub(unsigned char mode_name)
    // {
        // 동진이 형님이 개씹 ㅈㄴ 중요하다고 강조하신 mode : 꼭 공부!
    // }
 
    void twist_to_turning_radius(const geometry_msgs::Twist::ConstPtr& msg)
    {
        l = msg->linear.x / msg->angular.z;
    }

    void calculate_servo_angle(double l)
    {
        theta_front_closest = atan2(d3, abs(l) - d1);
        theta_front_farthest = atan2(d3, abs(l) + d1);

        if(l > 0)
        {
            FL_angle.data = theta_front_closest;
            FR_angle.data = theta_front_farthest;
            RL_angle.data = -theta_front_closest;
            RR_angle.data = -theta_front_farthest;
        }
        else
        {
            FL_angle.data = -theta_front_farthest;
            FR_angle.data = -theta_front_closest;
            RL_angle.data = theta_front_farthest;
            RR_angle.data = theta_front_closest;
        }
        
    }

    void calculate_drive_velocity(float velocity, double l)
    {
        angular_velocity_center = velocity / abs(l);

        vel_middle_closest = (abs(l) - d4) * angular_velocity_center;
        vel_corner_closest = hypot(abs(l) - d1, d3) * angular_velocity_center;
        vel_corner_farthest = hypot(abs(l) + d1, d3) * angular_velocity_center;
        vel_middle_farthest = (abs(l) + d4) * angular_velocity_center;

        ang_vel_middle_closest = vel_middle_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_closest = vel_corner_closest / ROVER_WHEEL_RADIUS;
        ang_vel_corner_farthest = vel_corner_farthest / ROVER_WHEEL_RADIUS;
        ang_vel_middle_farthest = vel_middle_farthest / ROVER_WHEEL_RADIUS;

        if (l > 0)  // turning left
        {
            FL_Wheel_Velocity.data = float(ang_vel_corner_closest);
            RL_Wheel_Velocity.data = float(ang_vel_corner_closest);
            ML_Wheel_Velocity.data = float(ang_vel_middle_closest);
            RR_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            FR_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            MR_Wheel_Velocity.data = float(ang_vel_middle_farthest);
        }
        else        // turning right
        {
            FL_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            RL_Wheel_Velocity.data = float(ang_vel_corner_farthest);
            ML_Wheel_Velocity.data = float(ang_vel_middle_farthest);
            RR_Wheel_Velocity.data = float(ang_vel_corner_closest);
            FR_Wheel_Velocity.data = float(ang_vel_corner_closest);
            MR_Wheel_Velocity.data = float(ang_vel_middle_closest);
        }

    }

    void go_straight(const geometry_msgs::Twist::ConstPtr& msg)
    {
        FL_Wheel_Velocity.data = float(msg->linear.x / ROVER_WHEEL_RADIUS);
        RL_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        ML_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        RR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        FR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        MR_Wheel_Velocity.data = FL_Wheel_Velocity.data;

        FL_angle.data = 0;
        FR_angle.data = 0;
        RL_angle.data = 0;
        RR_angle.data = 0;
    }

    void stop()
    {
        FL_Wheel_Velocity.data = 0;
        RL_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        ML_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        RR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        FR_Wheel_Velocity.data = FL_Wheel_Velocity.data;
        MR_Wheel_Velocity.data = FL_Wheel_Velocity.data;

        FL_angle.data = 0;
        FR_angle.data = 0;
        RL_angle.data = 0;
        RR_angle.data = 0;
    }

    void rotate_in_place(const geometry_msgs::Twist::ConstPtr& msg)
    {
        FL_Wheel_Velocity.data = -float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);
        FR_Wheel_Velocity.data = float(sqrt(d1*d1+d3*d3) * msg->angular.z / ROVER_WHEEL_RADIUS);
        ML_Wheel_Velocity.data = -float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        MR_Wheel_Velocity.data = float(d4 * msg->angular.z / ROVER_WHEEL_RADIUS);
        RL_Wheel_Velocity.data = -float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);
        RR_Wheel_Velocity.data = float(sqrt(d1*d1+d2*d2) * msg->angular.z / ROVER_WHEEL_RADIUS);
    }

    // 콜백 함수 : 
    void msgCallback(const geometry_msgs::Twist::ConstPtr& msg) // msgCallback 함수 정의
    {
        ROS_INFO("angular : %lf , linear : %lf", msg->angular.z, msg->linear.x);    // cmd_vel로 받은 선속도, 각속도를 출력
        
        if((msg->angular.z != 0)&&(msg->linear.x == 0))  // rotate in place
        {
            FL_angle.data = -atan(d3/d1);    // FL
            FR_angle.data = atan(d3/d1);     // FR
            RL_angle.data = atan(d2/d1);     // RL
            RR_angle.data = -atan(d2/d1);    // RR

            rotate_in_place(msg);
            publishAngles();                // 서보모터 각도 publish
            publishVelocity();
        }
        
        else if((msg->angular.z != 0)&&(msg->linear.x != 0)) // 회전
        {
            // 1. 회전반경 구하기
            twist_to_turning_radius(msg);

            // 2. 서보모터 각도 구하기
            calculate_servo_angle(l);

            // 3. 휠 각속도 구하기
            calculate_drive_velocity(msg->linear.x, l);

            // 4. publish
            publishAngles();
            publishVelocity();
        }
        else if((msg->angular.z == 0)&&(msg->linear.x != 0)) // 직진
        {
            go_straight(msg);
            publishAngles();
            publishVelocity();
        }
        else if((msg->angular.z == 0)&&(msg->linear.x == 0))
        {
            stop();
            publishAngles();
            publishVelocity();
        }
    }

    // imu 콜백 함수
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {

        start_time = ros::Time::now().toSec();
        
        time = start_time - pre_time;
        
        pre_time = start_time;

        printf("difference : %f\n", time);

        ROS_INFO("imuCallback\n");
        double angle, dl, dx, dy;
        // angle = rover가 바라보는 각도
        // dl = 단위시간(0.01s)동안 rover가 간 거리
        angle = 2 * atan2(msg->orientation.z, msg->orientation.w);
        dl = Odometry(angle);
        dx = dl*cos(angle);
        dy = dl*sin(angle);
        x = x + dx;
        y = y + dy;
        ROS_INFO("position : [%f, %f]\n", x, y);
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        fl_vel = double(msg->velocity[2]);
        fr_vel = double(msg->velocity[3]);
        ml_vel = double(msg->velocity[4]);
        mr_vel = double(msg->velocity[5]);
        rl_vel = double(msg->velocity[8]);
        rr_vel = double(msg->velocity[9]);

        printf("fl : %f   fr : %f\n", fl_vel, fr_vel);
        printf("ml : %f   mr : %f\n", ml_vel, mr_vel);
        printf("rl : %f   rr : %f\n", rl_vel, rr_vel);    
    }

    double Odometry(double angle)
    {
        // angle = rover가 바라보는 각도
        double dl;
        dl = (fl_vel + fr_vel + ml_vel + mr_vel + rl_vel + rr_vel) * ROVER_WHEEL_RADIUS * 1.1 * time / 6;   // 1.1을 곱한 이유는 오차 보정때문
        return dl;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");  // motor_controller라는 노드 생성

    Controller controller;

    ros::Rate loop_rate(100); //100ms

    while (ros::ok())
    {
        ros::spinOnce();

        //ROS_INFO("HELLO");

        // servo.publishAngles();

        loop_rate.sleep();

    }
    ros::spinOnce();

    return 0;
}

//odom을 계산할 때
//topic joint_states를 확인해서 position을 확인하기 이때 이 친구는 라디안임 