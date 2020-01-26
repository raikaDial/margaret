#include <mbed.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>

#include <EthernetInterface.h>

#include <BNO055.h>

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

int main()
{
    nh.initNode("192.168.1.5");
    nh.advertise(bno055_imu_pub);

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
        publishIMUData();
      }

      if(blink_timer.read_ms() > 500)
      {
          led1 = !led1;
          blink_timer.reset();

          pc.printf("Ros node connected to master? %d\n", nh.connected());
      }

      nh.spinOnce();
    }
}