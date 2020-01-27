///////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @brief Main file for Margaret STM32F767ZI firmware
///
/// @author Ryker Dial
/// @date Jan. 26, 2020
///////////////////////////////////////////////////////////////////////////

#include <mbed.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>

#include <BNO055.h>
#include <xv11_lidar.h>

// ***** Objects for ROS Communication ***** //
ros::NodeHandle nh; // Main interface with ROS

sensor_msgs::Imu bno055_imu_msg;
ros::Publisher bno055_imu_pub("imu", &bno055_imu_msg);
// ********** //

// User LEDs for Debugging
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

Serial pc(USBTX,USBRX,921600);

// ***** Setup BNO055 IMU ***** //
// Pins for I2C communication and reset
const PinName BNO055_SDA = PB_9;
const PinName BNO055_SCL = PB_8;
const PinName BNO055_RST = PE_13;

// Create IMU object
BNO055 imu(BNO055_SDA, BNO055_SCL, BNO055_RST, BNO055_G_CHIP_ADDR, MODE_IMU);

BNO055_QUATERNION_TypeDef orientation; // Used to store the current orientation
BNO055_GYRO_TypeDef angular_rates; // Stores the current angular velocity of the IMU
BNO055_ACC_TypeDef acceleration; // Stores the current accelerometer data of the IMU

uint32_t imu_messages_published = 0;
Timer imu_data_publish_timer;
// ********** //

// ***** Dynamixel Configuration and Object Containers ***** //
static const PinName AX12_TX = PD_5;
static const PinName AX12_RX = PD_6;
static const PinName AX12_TX_EN = PF_15;
// ********** //

///////////////////////////////////////////////////////////////////////////
/// @brief Read IMU data over I2C and publish it.
///////////////////////////////////////////////////////////////////////////
void publishIMUData()
{
    bno055_imu_msg.header.stamp = nh.now(); // Fetch data timestamp

    imu.get_gyro(&angular_rates);
    imu.get_acc(&acceleration);
    imu.get_quaternion(&orientation);

    bno055_imu_msg.linear_acceleration.x = acceleration.x;
    bno055_imu_msg.linear_acceleration.y = acceleration.y;
    bno055_imu_msg.linear_acceleration.z = acceleration.z;

    bno055_imu_msg.angular_velocity.x = angular_rates.x;
    bno055_imu_msg.angular_velocity.y = angular_rates.y;
    bno055_imu_msg.angular_velocity.z = angular_rates.z;

    bno055_imu_msg.orientation.x = orientation.x;
    bno055_imu_msg.orientation.y = orientation.y;
    bno055_imu_msg.orientation.z = orientation.z;
    bno055_imu_msg.orientation.w = orientation.w;

    bno055_imu_msg.header.seq = imu_messages_published++;
    bno055_imu_pub.publish(&bno055_imu_msg);  
}

// ***** XV11 Lidar Configuration and Object Containers ***** //

// Lidar Module 1
const PinName XV11_LIDAR_1_TX = PC_12; // TX5
const PinName XV11_LIDAR_1_RX = PD_2; // RX5
const PinName XV11_LIDAR_1_PWM = PE_5; // PWM9/1

// Lidar Module 2
const PinName XV11_LIDAR_2_TX = PE_8; // TX7
const PinName XV11_LIDAR_2_RX = PE_7; // RX7
const PinName XV11_LIDAR_2_PWM = PE_6; // PWM9/2

Xv11Lidar xv11_lidar_1(XV11_LIDAR_1_TX, XV11_LIDAR_1_RX, XV11_LIDAR_1_PWM, 1);
Xv11Lidar xv11_lidar_2(XV11_LIDAR_2_TX, XV11_LIDAR_2_RX, XV11_LIDAR_2_PWM, 2);

ros::Publisher xv11_laserscan_pub_1("scan_1", &xv11_lidar_1.m_laserscan_msg);
ros::Publisher xv11_laserscan_pub_2("scan_2", &xv11_lidar_2.m_laserscan_msg);
// *********** //

int main()
{
    nh.initNode("192.168.1.5");
    nh.advertise(bno055_imu_pub);
    nh.advertise(xv11_laserscan_pub_1);
    nh.advertise(xv11_laserscan_pub_2);

    // Spin up the LIDARs.
    xv11_lidar_1.begin();
    xv11_lidar_2.begin();

    // Periodically send messages over the socket.
    Timer blink_timer;
    blink_timer.reset();
    blink_timer.start();

    imu_data_publish_timer.reset();
    imu_data_publish_timer.start();

    __enable_irq();

    while(1)
    {
        if(imu_data_publish_timer.read_us() >= 10000)
        {
            imu_data_publish_timer.reset();
            // publishIMUData();
        }

        xv11_lidar_1.update();
        if(xv11_lidar_1.m_laserscan_ready)
        {
            pc.printf("New LaserScan 1\n");
            xv11_laserscan_pub_1.publish(&xv11_lidar_1.m_laserscan_msg);
            xv11_lidar_1.m_laserscan_ready = 0;
        }
        xv11_lidar_2.update();
        if(xv11_lidar_2.m_laserscan_ready)
        {
            //pc.printf("New LaserScan 2\n");
            led3 = !led3;
            xv11_laserscan_pub_2.publish(&xv11_lidar_2.m_laserscan_msg);
            xv11_lidar_2.m_laserscan_ready = 0;
        }

    if(blink_timer.read_ms() > 500)
    {
        led1 = !led1;
        blink_timer.reset();

        //pc.printf("Ros node connected to master? %d\n", nh.connected());
    }

      nh.spinOnce();
    }
}