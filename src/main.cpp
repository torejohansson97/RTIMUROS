#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "RTIMULib.h"


int main(int argc, char **argv){
	

	// RTIMU setup
	int sampleCount = 0;
	int sampleRate = 0;
	uint64_t rateTimer;
	uint64_t displayTimer;
	uint64_t now;

	RTIMUSettings *settings = new RTIMUSettings("RTIMUSettings");

	RTIMU *imu = RTIMU::createIMU(settings);

	if((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)){
		printf("No IMU found\n");
		exit(1);
	}

	imu->IMUInit();

	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(true);
	
	// ROS setup
	ros::init(argc, argv, "BerryIMU");
	ros::NodeHandle n;

	ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("BerryIMU", 100);

	ros::Rate loop_rate(10);
	
	rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();

	// Main loop
	while(ros::ok()){
		sensor_msgs::Imu imu_msgs;
		usleep(imu->IMUGetPollInterval() * 1000);

		while(imu->IMURead()){
			RTIMU_DATA imuData = imu->getIMUData();
			RTQuaternion imuQuaternion = imu->getMeasuredQPose();
			sampleCount++;

			now = RTMath::currentUSecsSinceEpoch();
			if(now - displayTimer > 1000000){
				imu_msgs.linear_acceleration.x = imuData.accel.x();
				imu_msgs.linear_acceleration.y = imuData.accel.y();
				imu_msgs.linear_acceleration.z = imuData.accel.z();

				imu_msgs.angular_velocity.x = imuData.gyro.x();
				imu_msgs.angular_velocity.y = imuData.gyro.y();
				imu_msgs.angular_velocity.z = imuData.gyro.z();

				imu_msgs.orientation.x = imuQuaternion.x();
				imu_msgs.orientation.y = imuQuaternion.y();
				imu_msgs.orientation.z = imuQuaternion.z();
				imu_msgs.orientation.w = imuQuaternion.scalar();
	
				IMU_pub.publish(imu_msgs);
				ROS_INFO("Sending IMU data. Sample rate: %i", sampleRate);
				displayTimer = now;
			}
			if(now - rateTimer > 1000000){
				sampleRate = sampleCount;
				sampleCount = 0;
				rateTimer = now;
			}
		}
		ros::spinOnce();
		//loop_rate.sleep();
	}
}
