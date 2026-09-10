
#include "src/msg_manager.h"
#include "rclcpp/rclcpp.hpp"

#include "src/common/cmdline.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char*> non_ros_argv;
    for (auto& arg : non_ros_args) {
        non_ros_argv.push_back(const_cast<char*>(arg.c_str()));
    }

    cmdline::parser g_parser;
    // g_parser.add<std::string>("config", 'c', "config file", true, "");
    g_parser.add("version", 'v', "print version");
    g_parser.add<int>("log-level", 'l', "", false, 4);
    g_parser.add<std::string>("output", 'o', "output log path", false, ".");
    g_parser.parse_check(non_ros_argv.size(), non_ros_argv.data());

    // check version
    if(g_parser.exist("version")) { 
        //usage
        // std::cout<< g_parser.usage() << std::endl; 
        return 0;
    }
    
    // 初始化ROS2
    try {
        // 设置日志输出目录，先从输入参数里获取，如果没有则默认为当前目录
        Logger::GetInstance().SetLogFilePath(g_parser.get<std::string>("output"));
        Logger::GetInstance().SetLogFileSize(10); // 设置日志文件大小，单位MB
        Logger::GetInstance().SetLogFileNum(5); // 设置日志文件个数，0表示不滚动单个文件记录
        // 日志等级，先从输入参数里获取，如果没有则默认为info
        auto log_level = (Logger::Level)g_parser.get<int>("log-level");
        Logger::GetInstance().Init("cannode", Logger::kRotatingFile | Logger::kConsole, log_level, log_level); // 初始化日志
    } catch (const std::exception& e) {
        // std::cout << "Failed to initialize logger: "<< e.what() << "\n";
        return 1;
    }

    // 创建底盘驱动节点
    auto msg_node = std::make_shared<cannode::MsgManager>();
    msg_node->init();

    // 运行节点
    rclcpp::spin(msg_node);
    rclcpp::shutdown();
    
    return 0;
}
