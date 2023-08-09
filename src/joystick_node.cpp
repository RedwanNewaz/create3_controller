//
// Created by roboticslab on 8/9/23.
//
#include "controller.hpp"
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("namespace,ns", po::value<std::vector<std::string>>()->multitoken(), "input vector of strings");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);


    vector<string> namespaces;

    if (vm.count("namespace")) {
        namespaces = vm["namespace"].as<std::vector<std::string>>();
    }

    auto joyManager = make_shared<controller::joystick>(namespaces);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(joyManager);
    executor.spin();
    return 0;
}
