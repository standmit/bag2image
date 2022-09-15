#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;


constexpr char OPT_TOPIC[]     = "topic";
constexpr char OPT_BAG[]       = "bag";
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
            std::cout << "extract_posewithcovariancestamped [options]";
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
    assert(view.begin()->getDataType() == ros::message_traits::datatype<geometry_msgs::PoseWithCovarianceStamped>());

    std::cout << "nsec";
    std::cout << "\tpos_x\tpos_y\tpos_z";
    std::cout << "\tquat_x\tquat_y\tquat_z\tquat_w";
    std::cout << "\tpose_cov";
    std::cout << std::endl;

    for (rosbag::MessageInstance const instance: view) {
        const auto msg = instance.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (msg == NULL)
            continue;
        
        std::ostringstream ss_sec, ss_nsec;
        ss_sec << std::setw(9) << std::setfill('0') << msg->header.stamp.sec;
        ss_nsec << std::setw(9) << std::setfill('0') << msg->header.stamp.nsec;
        std::cout << ss_sec.str() << ss_nsec.str();
        std::cout << '\t' << msg->pose.pose.position.x;
        std::cout << '\t' << msg->pose.pose.position.y;
        std::cout << '\t' << msg->pose.pose.position.z;
        std::cout << '\t' << msg->pose.pose.orientation.x;
        std::cout << '\t' << msg->pose.pose.orientation.y;
        std::cout << '\t' << msg->pose.pose.orientation.z;
        std::cout << '\t' << msg->pose.pose.orientation.w;
        std::cout << '\t' << msg->pose.covariance[0];
        for (int i = 1; i < msg->pose.covariance.size(); ++i)
            std::cout << ',' << msg->pose.covariance[i];
        std::cout << std::endl;
    }

    bag.close();
    return 0;
}
