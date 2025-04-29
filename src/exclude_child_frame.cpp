#include <algorithm>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <boost/program_options.hpp>


namespace po = boost::program_options;


constexpr char OPT_HELP[] = "help";
constexpr char OPT_INBAG[] = "inbag";
constexpr char OPT_OUTBAG[] = "outbag";
constexpr char OPT_FRAME[] = "frame";


int main(int argc, char** argv) {
    const po::variables_map vm = [] (const int argc, char** argv) -> const po::variables_map {
        po::options_description arguments("Arguments");
        arguments.add_options()
            (OPT_INBAG, po::value<std::string>(), "Input BAG-file")
            (OPT_OUTBAG, po::value<std::string>(), "Output BAG-file")
            (OPT_FRAME, po::value<std::string>(), "Frame to exclude")
        ;

        po::positional_options_description arguments_pos;
        arguments_pos.add(OPT_INBAG, 1);
        arguments_pos.add(OPT_OUTBAG, 1);
        arguments_pos.add(OPT_FRAME, 1);

        po::options_description general("General options");
        general.add_options()
            (OPT_HELP, "print this message")
        ;

        po::options_description all_options("All options");
        all_options.add(arguments).add(general);

        po::variables_map vm;
        po::store(
            po::command_line_parser(argc, argv).options(all_options).positional(arguments_pos).run(),
            vm
        );
        po::notify(vm);

        if (
                vm.count(OPT_HELP) != 0 ||
                vm.count(OPT_INBAG) != 1 ||
                vm.count(OPT_OUTBAG) != 1 ||
                vm.count(OPT_FRAME) != 1
        ) {
            std::cout << "exclude_child_frame [options]";
            std::cout << ' ' << OPT_INBAG;
            std::cout << ' ' << OPT_OUTBAG;
            std::cout << ' ' << OPT_FRAME;
            std::cout << std::endl << std::endl;
            std::cout << arguments << std::endl;
            std::cout << general << std::endl;
            exit(1);
        }

        return vm;
    } (argc, argv);

    rosbag::Bag inbag(vm[OPT_INBAG].as<std::string>(), rosbag::BagMode::Read);
    rosbag::Bag outbag(vm[OPT_OUTBAG].as<std::string>(), rosbag::BagMode::Write);
    rosbag::View view(inbag);
    const std::string exclude = vm[OPT_FRAME].as<std::string>();

    for (rosbag::MessageInstance const instance : view) {
        const auto& topic = instance.getTopic();
        if (topic == "/tf" || topic == "/tf_static") {
            const auto msg = instance.instantiate<tf2_msgs::TFMessage>();
            msg->transforms.erase(
                std::remove_if(
                    msg->transforms.begin(),
                    msg->transforms.end(),
                    [&exclude] (const geometry_msgs::TransformStamped& tf) {
                        return (tf.child_frame_id == exclude);
                    }
                ),
                msg->transforms.end()
            );
            if (msg->transforms.size() > 0)
                outbag.write(
                    topic,
                    instance.getTime(),
                    msg
                );
        } else {
            outbag.write(instance.getTopic(), instance.getTime(), instance);
        }
    }

    return 0;
}