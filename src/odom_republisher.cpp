#define POSE_POSITION_VARIANCE 99.0
#define POSE_ORIENTATION_VARIANCE 99999.0
#define TWIST_POSITION_VARIANCE 99.0
#define TWIST_ORIENTATION_VARIANCE 99999.0

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/**
 * 
 * @brief class for republishing wheelodometry with customizable covariances (additionally publishes tf between base_link and wheelodom frame)
 */
class OdomRepublisher
{
public:
	/**
	 *
	 * @brief initializes all the publisher/subscriber and loads rosparams
	 * @param nh ROS Node handler
	 */
	OdomRepublisher(ros::NodeHandle nh) : 
		nh_(nh)
	{
		odom_republisher_ = nh_.advertise<nav_msgs::Odometry>("/wheelodom_republished", 10);
		odom_subscriber_ = nh_.subscribe("/wheelodom", 10, &OdomRepublisher::odomCallback, this);

		ros::param::param<float>("~pose_position_variance",		pose_position_variance_,		POSE_POSITION_VARIANCE);
		ros::param::param<float>("~pose_orientation_variance",	pose_orientation_variance_,		POSE_ORIENTATION_VARIANCE);
		ros::param::param<float>("~twist_position_variance",	twist_position_variance_,		TWIST_POSITION_VARIANCE);
		ros::param::param<float>("~twist_orientation_variance",	twist_orientation_variance_,	TWIST_ORIENTATION_VARIANCE);

		ros::param::param<std::string>("~wheelodom_frame_id",	wheelodom_frame_id_, "wheelodom");

	}

	/**
	 * @brief subscriber callback for wheel odom. Republishes the wheelodom with desired covariances and publishes tf between base_link and wheelodom frame
	 * @param odom_msg the odometry message
	 * @return none
	 */
	void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
	{
		nav_msgs::Odometry new_odom = *odom_msg;
		new_odom.header.frame_id = wheelodom_frame_id_;
		
		new_odom.pose.covariance[0]		= pose_position_variance_;
		new_odom.pose.covariance[7]		= pose_position_variance_;
		new_odom.pose.covariance[14]	= pose_position_variance_;

		new_odom.pose.covariance[21]	= pose_orientation_variance_;
		new_odom.pose.covariance[28]	= pose_orientation_variance_;
		new_odom.pose.covariance[35]	= pose_orientation_variance_;

		new_odom.twist.covariance[0]	= twist_position_variance_;
		new_odom.twist.covariance[7]	= twist_position_variance_;
		new_odom.twist.covariance[14]	= twist_position_variance_;

		new_odom.twist.covariance[21]	= twist_orientation_variance_;
		new_odom.twist.covariance[28]	= twist_orientation_variance_;
		new_odom.twist.covariance[35]	= twist_orientation_variance_;

		odom_republisher_.publish(new_odom);

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(new_odom.pose.pose.position.x, new_odom.pose.pose.position.y, new_odom.pose.pose.position.z) );
		transform.setRotation(tf::Quaternion(
			new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y, new_odom.pose.pose.orientation.z, new_odom.pose.pose.orientation.w
		));
		// to avoid having multiple parent to base_link, send the wheelodom_frame transform as child of base_link
		tf_broadcaster_.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), new_odom.child_frame_id, wheelodom_frame_id_));

	}
	
private:
	ros::NodeHandle nh_;
	ros::Publisher odom_republisher_;
	ros::Subscriber odom_subscriber_;

	float pose_position_variance_;
	float pose_orientation_variance_;
	float twist_position_variance_;
	float twist_orientation_variance_;

	tf::TransformBroadcaster tf_broadcaster_;
	std::string wheelodom_frame_id_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_republisher");
	ros::NodeHandle nh("~");
	OdomRepublisher odom_republisher(nh);
	ros::spin();

	return 0;
}