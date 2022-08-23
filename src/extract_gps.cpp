#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;


constexpr char OPT_TOPIC[]     = "topic";
constexpr char OPT_BAG[]       = "bag";
constexpr char OPT_NOSTAMP[]   = "nostamp";
constexpr char OPT_HELP[]      = "help";


int main(int argc, char **argv) {
    po::variables_map vm;
    {
        po::options_description pos_desc_backend("Arguments");
        pos_desc_backend.add_options()
            (OPT_BAG, po::value<std::string>(), "BAG-file")
            (OPT_TOPIC, po::value<std::string>(), "topic name")
        ;

        po::positional_options_description pos_desc;
        pos_desc.add(OPT_BAG, 1);
        pos_desc.add(OPT_TOPIC, 1);

        po::options_description vis_desc("Options");
        vis_desc.add_options()
            (OPT_HELP, "print this message")
            (OPT_NOSTAMP, po::bool_switch(), "use bag stamp instead of message stamp")
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
                !vm.count(OPT_TOPIC)
        ) {
            std::cout << "extract_imu [options]";
            std::cout << " " << OPT_BAG;
            std::cout << " " << OPT_TOPIC;
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
    assert(view.begin()->getDataType() == ros::message_traits::datatype<sensor_msgs::NavSatFix>());
    const bool nostamp = vm[OPT_NOSTAMP].as<bool>();

    std::cout << "nsec\tlat\tlon\talt";
    std::cout << "\tcovariance";
    std::cout << std::endl;
    
    for (rosbag::MessageInstance const instance: view) {
        const auto msg = instance.instantiate<sensor_msgs::NavSatFix>();
        if (msg == NULL)
            continue;
        
        const auto& stamp = nostamp ? instance.getTime() : msg->header.stamp;
        std::ostringstream ss_sec, ss_nsec;
        ss_sec << std::setw(9) << std::setfill('0') << stamp.sec;
        ss_nsec << std::setw(9) << std::setfill('0') << stamp.nsec;
        std::cout << ss_sec.str() << ss_nsec.str();
        std::cout << '\t' << msg->latitude;
        std::cout << '\t' << msg->longitude;
        std::cout << '\t' << msg->altitude;
        std::cout << '\t' << msg->position_covariance[0];
        std::cout << ',' << msg->position_covariance[1];
        std::cout << ',' << msg->position_covariance[2];
        std::cout << ',' << msg->position_covariance[3];
        std::cout << ',' << msg->position_covariance[4];
        std::cout << ',' << msg->position_covariance[5];
        std::cout << ',' << msg->position_covariance[6];
        std::cout << ',' << msg->position_covariance[7];
        std::cout << ',' << msg->position_covariance[8];
        std::cout << std::endl;
    }

    bag.close();
    return 0;
}
