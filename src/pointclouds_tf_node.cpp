#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>

class PointCloudsTransformer
{
private:
    ros::Subscriber sub;
    ros::Publisher pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    void
    cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
    {
        // Define the source and target frames
        std::string source_frame = cloud_in->header.frame_id;
        std::string target_frame = "rover";

        // Get the transform from the source frame to the target frame
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        tf::StampedTransform tf_transform;
        tf::transformStampedMsgToTF(transform, tf_transform);

        // Transform the point cloud
        sensor_msgs::PointCloud2 cloud_out;
        pcl_ros::transformPointCloud(target_frame, tf_transform, *cloud_in, cloud_out);

        cloud_out.header.stamp = ros::Time::now();
        cloud_out.header.frame_id = target_frame;
        pub.publish(cloud_out);
    };

public:
    PointCloudsTransformer(ros::NodeHandle &nh);
    ~PointCloudsTransformer();
};

PointCloudsTransformer::PointCloudsTransformer(ros::NodeHandle &nh) : tf_listener(tf_buffer)
{
    std::string ns = ros::this_node::getNamespace();

    sub = nh.subscribe<sensor_msgs::PointCloud2>(ns + "/ouster/points", 1, &PointCloudsTransformer::cloudCallback, this);

    pub = nh.advertise<sensor_msgs::PointCloud2>(ns + "/ouster/transformed_points", 1);
}

PointCloudsTransformer::~PointCloudsTransformer()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_point_cloud");
    ros::NodeHandle nh;

    PointCloudsTransformer ouster_transformer(nh);

    ros::spin();
    return 0;
}