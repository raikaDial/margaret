#define STM32F767ZI // Define so rosserial switches to using ethernet

#include <Arduino.h>

#include <ros.h>
#include <sensor_msgs/IMU.h>

#include <Adafruit_BNO055.h>

ros::NodeHandle  nh;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
sensor_msgs::Imu bno055_imu_msg;
ros::Publisher bno055_imu_pub("imu", &bno055_imu_msg);
uint32_t imu_messages_published = 0;

void setup()
{
  bno055_imu_msg.header.frame_id = "imu_link";
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS); // Connect to IMU

  nh.initNode();
  nh.advertise(bno055_imu_pub);
}

void loop()
{
  bno055_imu_msg.header.stamp = nh.now(); // Fetch data timestamp

  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> angular_rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion orientation = bno.getQuat();

  bno055_imu_msg.linear_acceleration.x = acceleration.x();
  bno055_imu_msg.linear_acceleration.y = acceleration.y();
  bno055_imu_msg.linear_acceleration.z = acceleration.z();

  bno055_imu_msg.angular_velocity.x = angular_rates.x();
  bno055_imu_msg.angular_velocity.y = angular_rates.y();
  bno055_imu_msg.angular_velocity.z = angular_rates.z();

  bno055_imu_msg.orientation.x = orientation.x();
  bno055_imu_msg.orientation.y = orientation.y();
  bno055_imu_msg.orientation.z = orientation.z();
  bno055_imu_msg.orientation.w = orientation.w();

  bno055_imu_msg.header.seq = imu_messages_published++;
  bno055_imu_pub.publish(&bno055_imu_msg);
  nh.spinOnce();
  delay(10);
}
