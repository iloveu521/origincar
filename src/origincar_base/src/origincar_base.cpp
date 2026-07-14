#include "origincar_base/origincar_base.h"
#include "rclcpp/rclcpp.hpp"
#include "origincar_base/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "origincar_msg/msg/data.hpp"

using std::placeholders::_1;
using namespace std;
void sigintHandler(int sig);
sensor_msgs::msg::Imu Mpu6050;
rclcpp::Node::SharedPtr node_handle = nullptr;

// IMU 零偏校准值 (来自 imu_calibrate.py, 静置92.3s, 1847采样点)
static const double GYRO_BIAS_X  = -0.00552874;  // 陀螺仪 X 轴零偏 (rad/s)
static const double GYRO_BIAS_Y  = +0.00320146;  // 陀螺仪 Y 轴零偏 (rad/s)
static const double GYRO_BIAS_Z  = -0.00025274;  // 陀螺仪 Z 轴零偏 (rad/s)
static const double ACCEL_BIAS_X = +0.20657806;  // 加速度计 X 轴零偏 (m/s²)
static const double ACCEL_BIAS_Y = +0.03238653;  // 加速度计 Y 轴零偏 (m/s²)
static const double ACCEL_BIAS_Z = -0.15839675;  // 加速度计 Z 轴零偏 (m/s²)

// 里程计校准系数 (自走0.905m实测, odom=0.9864m, 误差+9.0%)
static const double ODOM_LINEAR_SCALE_X = 0.944997;   // X 方向线性位移系数
static const double ODOM_LINEAR_SCALE_Y = 1.032157;   // Y 方向线性位移系数
static const double ODOM_ANGULAR_SCALE   = 1.0;        // 角速度系数 (待标定)


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    origincar_base Robot_Control;
    Robot_Control.Control();
    rclcpp::shutdown();
    return 0;
}

short origincar_base::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    return transition_16;
}

float origincar_base::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;
    transition_16 |=  Data_Low;
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001;
    return data_return;
  }

void origincar_base::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
    short  transition;
  
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = akm_ctl->drive.speed*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = akm_ctl->drive.steering_angle*1000/2;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); 
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void origincar_base::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short  transition;
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0; 

    transition=0;
    transition = twist_aux->linear.x*1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition>>8;

    transition=0;
    transition = twist_aux->linear.y*1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition>>8;

    transition=0;
    transition = (twist_aux->angular.z)*1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);
    Send_Data.tx[10]=FRAME_TAIL;

    try {
      if (akm_cmd_vel == "none") {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
      } 
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port"));
    }
}

void origincar_base::Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch)
{
  (void)sign_switch;
    /* if (sign_switch->data == -1) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 0.5;
         Robot_Pos.Y = 0.2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }
     else if (sign_switch->data == 6) {
         memset(&Robot_Pos, 0, sizeof(Robot_Pos));
         Robot_Pos.X = 2;
         Robot_Pos.Y = 2;
         memset(&Robot_Vel, 0, sizeof(Robot_Vel));
     }*/
}

void origincar_base::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;
    Imu_Data_Pub.header.stamp = rclcpp::Node::now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 
                                                  
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6; 
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    // 三轴陀螺仪噪声相同（std≈0.0008 rad/s → variance≈1e-6）
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e-6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e-6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    // 加速度计噪声（std≈0.017 m/s² → variance≈1e-3）
    Imu_Data_Pub.linear_acceleration_covariance[0] = 1e-3;
    Imu_Data_Pub.linear_acceleration_covariance[4] = 1e-3;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 1e-3;

    imu_publisher->publish(Imu_Data_Pub);

}

void origincar_base::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    
    origincar_msg::msg::Data robotpose;
    origincar_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id;

    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;

    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;


    odom.twist.twist.linear.x =  Robot_Vel.X;
    odom.twist.twist.linear.y =  Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z; 

    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    odom_publisher->publish(odom);
    robotpose_publisher->publish(robotpose);
    robotvel_publisher->publish(robotvel); 
}

void origincar_base::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;

    if (Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;
        voltage_publisher->publish(voltage_msgs);
    }
}

unsigned char origincar_base::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0) {
      for(k=0; k < Count_Number; k++) {
        check_sum = check_sum^Receive_Data.rx[k];
      }
    } else if (mode == 1) {
      for (k=0; k < Count_Number; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    }

    return check_sum;
}

bool origincar_base::Get_Sensor_Data()
{
    short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0;
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0}; 
    Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr)); 
    for (j = 0; j < 24; j++) {
      if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos=j;
      else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
    }

    if (Tail_Pos == (Header_Pos + 23)) {
      memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }  else if (Header_Pos == (1 + Tail_Pos)) {
      for (j = 0;j < 24; j++)
      Receive_Data.rx[j] = Receive_Data_Pr[(j+Header_Pos) % 24];
    }  else {
    return false;
    }

    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];
    if (Receive_Data.Frame_Header == FRAME_HEADER) {
      if (Receive_Data.Frame_Tail == FRAME_TAIL) {
        if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)||(Header_Pos == (1 + Tail_Pos))) {
          Receive_Data.Flag_Stop=Receive_Data.rx[1];
          Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]);
        
          Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]);
                                                                          
          Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); 

          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);
          Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);
          Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);
          Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);

          // 原始值转换 + 零偏补偿
          Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO - ACCEL_BIAS_X;
          Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO - ACCEL_BIAS_Y;
          Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO - ACCEL_BIAS_Z;

          Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO - GYRO_BIAS_X;
          Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO - GYRO_BIAS_Y;
          Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO - GYRO_BIAS_Z;

          transition_16 = 0;
          transition_16 |=  Receive_Data.rx[20]<<8;
          transition_16 |=  Receive_Data.rx[21];
          Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;

          return true;
        }
      }
    }

    return false;
}

void origincar_base::Control()
{
    rclcpp::Time current_time, last_time;
    current_time = rclcpp::Node::now();
    last_time = rclcpp::Node::now();
    while(rclcpp::ok()) {
      current_time = rclcpp::Node::now();
      Sampling_Time = (current_time - last_time).seconds();
      if (true == Get_Sensor_Data()) {
        // 里程计积分 (带校准系数)
        Robot_Pos.X += ODOM_LINEAR_SCALE_X * (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
        Robot_Pos.Y += ODOM_LINEAR_SCALE_Y * (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
        Robot_Pos.Z += ODOM_ANGULAR_SCALE * Robot_Vel.Z * Sampling_Time;

        Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                  Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
        Publish_ImuSensor();
        Publish_Voltage();
        Publish_Odom();
        rclcpp::spin_some(this->get_node_base_interface());
      }
      last_time = current_time;
    }
}

origincar_base::origincar_base()
: rclcpp::Node ("origincar_base")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200;

  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

  robotpose_publisher = create_publisher<origincar_msg::msg::Data>("robotpose", 10);

  robotvel_publisher = create_publisher<origincar_msg::msg::Data>("robotvel", 10);

  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 1, std::bind(&origincar_base::Cmd_Vel_Callback, this, _1));
  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 1, std::bind(&origincar_base::Akm_Cmd_Vel_Callback, this, _1));

  // Sign_Switch_Sub = create_subscription<std_msgs::msg::Int32>(
  //     "/sign4return", 1, std::bind(&origincar_base::Sign_Switch_Callback, this, _1));
  try  {
    Stm32_Serial.setPort("/dev/ttyACM0");
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(),"origincar_base can not open serial port,Please check the serial port cable! ");
  }
  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(),"origincar_base serial port opened");
  }
}


void sigintHandler(int sig)
{
    sig = sig;
      printf("OriginBot shutdown...\n");
    serial::Serial Stm32_Serial;
    Stm32_Serial.setPort("/dev/ttyACM0");
    Stm32_Serial.setBaudrate(115200);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();                                       
    SEND_DATA Send_Data;
    if (Stm32_Serial.isOpen()) {
    Send_Data.tx[0]=FRAME_HEADER;
    Send_Data.tx[1] = 0;
    Send_Data.tx[2] = 0;

    Send_Data.tx[4] = 0;
    Send_Data.tx[3] = 0;

    Send_Data.tx[6] = 0;
    Send_Data.tx[5] = 0;

    Send_Data.tx[7] = 0;
    Send_Data.tx[8] = 0;
    int check_sum = 0;
    for (int k = 0; k < 9; k++) {
        check_sum = check_sum^Send_Data.tx[k];
      }
    Send_Data.tx[9]=check_sum;
    Send_Data.tx[10]=FRAME_TAIL;

    try {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
    }

  }
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();
}

origincar_base::~origincar_base()
{
  RCLCPP_INFO(this->get_logger(),"Shutting down");
}