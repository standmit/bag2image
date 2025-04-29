#include <boost/program_options.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


namespace po = boost::program_options;


constexpr char OPT_BAG[] = "bag";
constexpr char OPT_TOPIC[] = "topic";
constexpr char OPT_HELP[] = "help";


int main(int argc, char** argv) {
    const po::variables_map vm = [] (const int argc, char** argv) {
        po::variables_map vm;

        po::options_description arguments("Arguments");
        arguments.add_options()
            (OPT_BAG, po::value<std::string>(), "BAG-file")
            (OPT_TOPIC, po::value<std::string>(), "topic name")
        ;

        po::positional_options_description arguments_positions;
        arguments_positions.add(OPT_BAG, 1);
        arguments_positions.add(OPT_TOPIC, 1);

        po::options_description general("General options");
        general.add_options()
            (OPT_HELP, "print this message")
        ;

        po::options_description all_options("All options");
        all_options.add(arguments).add(general);

        po::store(
            po::command_line_parser(argc, argv).options(all_options).positional(arguments_positions).run(),
            vm
        );
        po::notify(vm);

        if (
                vm.count(OPT_HELP) != 0 ||
                vm.count(OPT_BAG) != 1 ||
                vm.count(OPT_TOPIC) != 1
        ) {
            std::cout << "extract_pc2 [options]";
            std::cout << " " << OPT_BAG;
            std::cout << " " << OPT_TOPIC;
            std::cout << std::endl << std::endl;
            std::cout << arguments << std::endl;
            std::cout << general << std::endl;
            exit(1);
        }

        return vm;
    } (argc, argv);

    rosbag::Bag bag(vm[OPT_BAG].as<std::string>(), rosbag::BagMode::Read);
    rosbag::View view(
        bag,
        rosbag::TopicQuery(
            vm[OPT_TOPIC].as<std::string>()
        )
    );
    assert(view.begin()->getDataType() == ros::message_traits::datatype<sensor_msgs::PointCloud2>());
    assert(view.begin()->getMD5Sum() == ros::message_traits::md5sum<sensor_msgs::PointCloud2>());

    std::cout << "recieve\ttimestamp";
    std::cout << "\tx\ty";
    std::cout << "\txq\tyq\tzq\twq";
    std::cout << "\tdist";
    std::cout << std::endl;

    for (rosbag::MessageInstance const instance : view) {
        const auto msg = instance.instantiate<sensor_msgs::PointCloud2>();
        if (msg == NULL)
            continue;
        
        sensor_msgs::PointCloud2ConstIterator<double> x_iter(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<double> y_iter(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<double> xq_iter(*msg, "xq");
        sensor_msgs::PointCloud2ConstIterator<double> yq_iter(*msg, "yq");
        sensor_msgs::PointCloud2ConstIterator<double> zq_iter(*msg, "zq");
        sensor_msgs::PointCloud2ConstIterator<double> wq_iter(*msg, "wq");
        sensor_msgs::PointCloud2ConstIterator<double> dist_iter(*msg, "dist");

        std::ostringstream recv_sec, recv_nsec, stamp_sec, stamp_nsec;
        recv_sec << std::setw(9) << std::setfill('0') << instance.getTime().sec;
        recv_nsec << std::setw(9) << std::setfill('0') << instance.getTime().nsec;
        stamp_sec << std::setw(9) << std::setfill('0') << msg->header.stamp.sec;
        stamp_nsec << std::setw(9) << std::setfill('0') << msg->header.stamp.nsec;

        while (x_iter != x_iter.end()) {
            std::cout << recv_sec.str() << recv_nsec.str();
            std::cout << '\t' << stamp_sec.str() << stamp_nsec.str();
            std::cout << '\t' << *x_iter;
            std::cout << '\t' << *y_iter;
            std::cout << '\t' << *xq_iter;
            std::cout << '\t' << *yq_iter;
            std::cout << '\t' << *zq_iter;
            std::cout << '\t' << *wq_iter;
            std::cout << '\t' << *dist_iter;
            std::cout << std::endl;

            ++x_iter;
            ++y_iter;
            ++xq_iter;
            ++yq_iter;
            ++zq_iter;
            ++wq_iter;
            ++dist_iter;
        }
    }

    return 0;
}