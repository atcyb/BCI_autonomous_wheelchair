#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("/imu_data", &imu_msg);

void setup()
{
  Wire.begin();
  bno.begin();
  nh.initNode();
  nh.advertise(imu_pub);
  delay(1000);
}

void loop()
{
  sensors_event_t event;
  bno.getEvent(&event);

  imu::Quaternion quat = bno.getQuat();
  
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "map"; // Parent frame
  imu_msg.child_frame_id = "imu_link";
  
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

    // Get angular velocity
  imu::Vector<3> ang_velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_msg.angular_velocity.x = ang_velocity.x();
  imu_msg.angular_velocity.y = ang_velocity.y();
  imu_msg.angular_velocity.z = ang_velocity.z();
  
  // Get linear acceleration
  imu::Vector<3> lin_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_msg.linear_acceleration.x = lin_acceleration.x();
  imu_msg.linear_acceleration.y = lin_acceleration.y();
  imu_msg.linear_acceleration.z = lin_acceleration.z();
  
  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
