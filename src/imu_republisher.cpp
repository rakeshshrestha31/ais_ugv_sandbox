#define ORIENTATION_COVARIANCE 99.0
#define ANGULAR_VELOCITY_COVARIANCE 99.0
#define LINEAR_ACCELERATION_COVARIANCE 9999.0

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

/**
 * 
 * @brief class for republishing imu topic with customizable covariances
 */
class ImuRepublisher
{
public:
	/**
	 *
	 * @brief initializes all the publisher/subscriber and loads rosparams
	 * @param nh ROS Node handler
	 */
	ImuRepublisher(ros::NodeHandle nh) : 
		nh_(nh)
	{
		imu_republisher_ = nh_.advertise<sensor_msgs::Imu>("/imu_republished", 10);
		imu_subscriber_ = nh_.subscribe("/au_Imu", 10, &ImuRepublisher::imuCallback, this);

		ros::param::param<float>("~orientation_variance",		orientation_variance_,			ORIENTATION_COVARIANCE);
		ros::param::param<float>("~angular_velocity_variance", 	angular_velocity_variance_,		ANGULAR_VELOCITY_COVARIANCE);
		ros::param::param<float>("~linear_acceleration_variance",linear_acceleration_variance_,	LINEAR_ACCELERATION_COVARIANCE);

		ros::param::param<float>("~accelerometer_x_bias", accelerometer_x_bias_, 0);
		ros::param::param<float>("~accelerometer_y_bias", accelerometer_y_bias_, 0);
		ros::param::param<float>("~accelerometer_z_bias", accelerometer_z_bias_, 0);

		ROS_INFO("accelerometer biases: %f, %f, %f", accelerometer_x_bias_, accelerometer_y_bias_, accelerometer_z_bias_);
	}

	/**
	 * @brief subscriber callback for Imu. Republishes the imu_msg with desired covariances and removes biases from raw IMU readings
	 * @param imu_msg the IMU message
	 * @return none
	 */
	void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
	{
		sensor_msgs::Imu new_imu = *imu_msg;

		new_imu.linear_acceleration.x -= accelerometer_x_bias_;
		new_imu.linear_acceleration.y -= accelerometer_y_bias_;
		new_imu.linear_acceleration.z -= accelerometer_z_bias_;
		
		new_imu.orientation_covariance[0]	= orientation_variance_;
		new_imu.orientation_covariance[4]	= orientation_variance_;
		new_imu.orientation_covariance[8]	= orientation_variance_;

		new_imu.angular_velocity_covariance[0]	= angular_velocity_variance_;
		new_imu.angular_velocity_covariance[4]	= angular_velocity_variance_;
		new_imu.angular_velocity_covariance[8]	= angular_velocity_variance_;

		new_imu.linear_acceleration_covariance[0]	= linear_acceleration_variance_;
		new_imu.linear_acceleration_covariance[4]	= linear_acceleration_variance_;
		new_imu.linear_acceleration_covariance[8] = linear_acceleration_variance_;

		imu_republisher_.publish(new_imu);
	}
	
private:
	ros::NodeHandle nh_;
	ros::Publisher imu_republisher_;
	ros::Subscriber imu_subscriber_;

	float orientation_variance_;
	float angular_velocity_variance_;
	float linear_acceleration_variance_;

	float accelerometer_x_bias_;
	float accelerometer_y_bias_;
	float accelerometer_z_bias_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_republisher");
	ros::NodeHandle nh("~");
	ImuRepublisher imu_republisher(nh);
	ros::spin();

	return 0;
}