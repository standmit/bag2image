#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "extract_images");
    ros::NodeHandle nh("~");
    if (not nh.hasParam("file")) {
        ROS_FATAL("File not specified");
        return 1;
    }
    if (not nh.hasParam("topic")) {
        ROS_FATAL("Topic not specified");
        return 2;
    }
    rosbag::Bag bag(nh.param<std::string>("file", ""), rosbag::bagmode::Read);
    std::vector<std::string> topics;
    const std::string topic_name=nh.param<std::string>("topic", "");
    topics.emplace_back(topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    {
        const std::string& topic_type = view.begin()->getDataType();
        if (topic_type != "sensor_msgs/CompressedImage") {
            ROS_FATAL("Topic %s has unsupported type %s", topic_name.c_str(), topic_type.c_str());
            return 3;
        }
    }

    const uint64_t total = view.size();
    const double total_f = static_cast<double>(total);
    uint64_t corrupted = 0;
    uint64_t i = 0;
    for (rosbag::MessageInstance const instance: view) {
        if (not ros::ok())
            break;
        i++;
        sensor_msgs::CompressedImage::ConstPtr msg = instance.instantiate<sensor_msgs::CompressedImage>();
        if (msg != NULL) {
            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg);
            std::ostringstream ss;
            ss << std::setw(9) << std::setfill('0') << msg->header.stamp.nsec;
            cv::imwrite(std::to_string(msg->header.stamp.sec) + ss.str() + ".jpg", cv_image->image);
        } else {
            ROS_WARN("Corrupted message");
            corrupted++;
            continue;
        }
        ROS_INFO_DELAYED_THROTTLE(1.0, "%lu/%lu (%3.1f%%)", i, total, static_cast<double>(i) / static_cast<double>(total) * 100.0);
    }
    ROS_INFO("%lu images extracted, %lu corrupted", total - corrupted, corrupted);

    bag.close();
    return 0;
}
