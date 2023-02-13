#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>


namespace po = boost::program_options;


constexpr char OPT_TOPIC[]     = "topic";
constexpr char OPT_DIRECTORY[] = "directory";
constexpr char OPT_BAG[]       = "bag";
constexpr char OPT_HELP[]      = "help";


namespace enc = sensor_msgs::image_encodings;


int main(int argc, char **argv) {
    po::variables_map vm;
    {
        po::options_description pos_desc_backend("Arguments");
        pos_desc_backend.add_options()
            (OPT_BAG, po::value<std::string>(), "BAG-file")
            (OPT_TOPIC, po::value<std::string>(), "topic name")
            (OPT_DIRECTORY, po::value<std::string>(), "output directory")
        ;

        po::positional_options_description pos_desc;
        pos_desc.add(OPT_BAG, 1);
        pos_desc.add(OPT_TOPIC, 1);
        pos_desc.add(OPT_DIRECTORY, 1);

        po::options_description vis_desc("Options");
        vis_desc.add_options()
            (OPT_HELP, "print this message")
        ;

        po::options_description all_desc("All options");
        all_desc.add(vis_desc).add(pos_desc_backend);

        po::store(
            po::command_line_parser(argc, argv).options(all_desc).positional(pos_desc).run(),
            vm
        );
        po::notify(vm);

        if (
                vm.count(OPT_HELP) ||
                !vm.count(OPT_BAG) ||
                !vm.count(OPT_TOPIC) ||
                !vm.count(OPT_DIRECTORY)
        ) {
            std::cout << "extract_images [options]";
            std::cout << " " << OPT_BAG;
            std::cout << " " << OPT_TOPIC;
            std::cout << " " << OPT_DIRECTORY;
            std::cout << std::endl << std::endl;
            std::cout << pos_desc_backend << std::endl;
            std::cout << vis_desc << std::endl;
            return 0;
        }
    }
    rosbag::Bag bag(vm[OPT_BAG].as<std::string>(), rosbag::bagmode::Read);
    std::vector<std::string> topics;
    const std::string topic_name = vm[OPT_TOPIC].as<std::string>();
    topics.emplace_back(topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    bool compressed;
    {
        const std::string& topic_type = view.begin()->getDataType();

        if (topic_type == "sensor_msgs/Image") {
            compressed = false;
        } else if (topic_type == "sensor_msgs/CompressedImage") {
            compressed = true;
        } else {
            printf("Topic %s has unsupported type %s\n", topic_name.c_str(), topic_type.c_str());
            return 3;
        }
    }

    const uint64_t total = view.size();
    const double total_f = static_cast<double>(total);
    uint64_t i = 0;
    const std::string output_dir = vm[OPT_DIRECTORY].as<std::string>();
    for (rosbag::MessageInstance const instance: view) {
        i++;
        cv::Mat image;
        std::string encoding;
        ros::Time stamp;
        if (compressed) {
            const auto msg = instance.instantiate<sensor_msgs::CompressedImage>();
            image = cv::imdecode(
                cv::Mat_<uchar>(
                    1,
                    msg->data.size(),
                    const_cast<uchar*>(&(msg->data[0]))
                ),
                cv::IMREAD_UNCHANGED
            );
            encoding = msg->format.substr(0, msg->format.find(';'));
            stamp = msg->header.stamp;
        } else {
            const auto msg = instance.instantiate<sensor_msgs::Image>();
            image = cv_bridge::toCvCopy(msg)->image;
            encoding = msg->encoding;
            stamp = msg->header.stamp;
        }
        if (enc::isBayer(encoding)) {
            int code = -1;
            if (encoding == enc::BAYER_RGGB8 || encoding == enc::BAYER_RGGB16)
                code = cv::COLOR_BayerBG2BGR;
            else if (encoding == enc::BAYER_BGGR8 || encoding == enc::BAYER_BGGR16)
                code = cv::COLOR_BayerRG2BGR;
            else if (encoding == enc::BAYER_GBRG8 || encoding == enc::BAYER_GBRG16)
                code = cv::COLOR_BayerGR2BGR;
            else if (encoding == enc::BAYER_GRBG8 || encoding == enc::BAYER_GRBG16)
                code = cv::COLOR_BayerGB2BGR;
            cv::cvtColor(image, image, code);
        }
        std::ostringstream ss_sec, ss_nsec;
        ss_sec << std::setw(9) << std::setfill('0') << stamp.sec;
        ss_nsec << std::setw(9) << std::setfill('0') << stamp.nsec;
        cv::imwrite(output_dir + "/" + ss_sec.str() + ss_nsec.str() + ".png", image);
        printf("\r%lu/%lu (%3.1f%%)", i, total, static_cast<float>(i) / static_cast<float>(total) * 100.f);
    }
    printf("\n%lu images extracted\n", total);

    bag.close();
    return 0;
}
