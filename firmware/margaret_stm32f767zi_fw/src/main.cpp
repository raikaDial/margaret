///////////////////////////////////////////////////////////////////////////
/// @file main.cpp
/// @brief Main file for Margaret STM32F767ZI firmware
///
/// @author Ryker Dial
/// @date Jan. 26, 2020
///////////////////////////////////////////////////////////////////////////

#include <mbed.h>

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include <BNO055.h>
#include <xv11_lidar.h>

// ***** Objects for ROS Communication ***** //
ros::NodeHandle nh; // Main interface with ROS
// ********** //

// User LEDs for Debugging
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

Serial pc(USBTX,USBRX,921600);
EventFlags event_flags;

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

std_msgs::UInt8MultiArray bno055_raw_imu_msg;
ros::Publisher bno055_raw_imu_pub("imu_raw", &bno055_raw_imu_msg);
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
    ros::Time timestamp = nh.now(); // Fetch data timestamp

    BNO055_UINT16_VEC3_TypeDef angular_rates;
    BNO055_UINT16_VEC3_TypeDef acceleration;
    BNO055_UINT16_VEC4_TypeDef orientation;

    imu.get_imu_data(&acceleration, &angular_rates, &orientation);

    // Store header info (timestamp and sequence no.)
    for(int i=0; i<4; ++i)
    {
        bno055_raw_imu_msg.data[3-i] = 0xFF & (timestamp.sec >> 8*i);
        bno055_raw_imu_msg.data[7-i] = 0xFF & (timestamp.nsec >> 8*i);
        bno055_raw_imu_msg.data[11-i] = 0xFF & (imu_messages_published >> 8*i);
    }

    // Store accelerations
    bno055_raw_imu_msg.data[12] = 0xFF & (acceleration.x >> 8);
    bno055_raw_imu_msg.data[13] = 0xFF & acceleration.x;
    bno055_raw_imu_msg.data[14] = 0xFF & (acceleration.y >> 8);
    bno055_raw_imu_msg.data[15] = 0xFF & acceleration.y;
    bno055_raw_imu_msg.data[16] = 0xFF & (acceleration.z >> 8);
    bno055_raw_imu_msg.data[17] = 0xFF & acceleration.z;

    // Store angular rates
    bno055_raw_imu_msg.data[18] = 0xFF & (angular_rates.x >> 8);
    bno055_raw_imu_msg.data[19] = 0xFF & angular_rates.x;
    bno055_raw_imu_msg.data[20] = 0xFF & (angular_rates.y >> 8);
    bno055_raw_imu_msg.data[21] = 0xFF & angular_rates.y;
    bno055_raw_imu_msg.data[22] = 0xFF & (angular_rates.z >> 8);
    bno055_raw_imu_msg.data[23] = 0xFF & angular_rates.z;

    // Store orientation
    bno055_raw_imu_msg.data[24] = 0xFF & (orientation.x >> 8);
    bno055_raw_imu_msg.data[25] = 0xFF & orientation.x;
    bno055_raw_imu_msg.data[26] = 0xFF & (orientation.y >> 8);
    bno055_raw_imu_msg.data[27] = 0xFF & orientation.y;
    bno055_raw_imu_msg.data[28] = 0xFF & (orientation.z >> 8);
    bno055_raw_imu_msg.data[29] = 0xFF & orientation.z;
    bno055_raw_imu_msg.data[30] = 0xFF & (orientation.w >> 8);
    bno055_raw_imu_msg.data[31] = 0xFF & orientation.w;

    bno055_raw_imu_pub.publish(&bno055_raw_imu_msg);

    imu_messages_published++;
}

void publishIMUDataTickerCallback()
{
    event_flags.set(1UL<<2);
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

ros::Publisher xv11_laserscan_pub_1("scan_raw_1", &xv11_lidar_1.m_laserscan_raw_msg);
ros::Publisher xv11_laserscan_pub_2("scan_raw_2", &xv11_lidar_2.m_laserscan_raw_msg);
// *********** //

int main()
{
    nh.initNode("192.168.1.5");

    bno055_raw_imu_msg.data_length = 32;
    bno055_raw_imu_msg.data = new uint8_t[bno055_raw_imu_msg.data_length];
    nh.advertise(bno055_raw_imu_pub);

    nh.advertise(xv11_laserscan_pub_1);
    nh.advertise(xv11_laserscan_pub_2);

    // Spin up the LIDARs.
    xv11_lidar_1.begin();
    xv11_lidar_2.begin();

    // Periodically send messages over the socket.
    Timer blink_timer;
    blink_timer.reset();
    blink_timer.start();

    Ticker imu_data_publish_ticker;
    imu_data_publish_ticker.attach_us(publishIMUDataTickerCallback, 10000);

    __enable_irq();

    while(1)
    {
        event_flags.wait_any(1UL, osWaitForever, false);

        if(event_flags.get() & 1UL)
        {
            xv11_lidar_1.update();
            if(xv11_lidar_1.m_laserscan_ready)
            {
                xv11_laserscan_pub_1.publish(&xv11_lidar_1.m_laserscan_raw_msg);
                xv11_lidar_1.m_laserscan_ready = 0;
            }
            event_flags.clear(1UL); 
        }

        if(event_flags.get() & (1UL<<1))
        {
            xv11_lidar_2.update();
            if(xv11_lidar_2.m_laserscan_ready)
            {
                xv11_laserscan_pub_2.publish(&xv11_lidar_2.m_laserscan_raw_msg);
                xv11_lidar_2.m_laserscan_ready = 0;
            }
            event_flags.clear(1UL<<1); 
        }

        if(event_flags.get() & (1UL<<2))
        {
            publishIMUData();
            event_flags.clear(1UL<<2);
        }

        nh.spinOnce();
    }
}