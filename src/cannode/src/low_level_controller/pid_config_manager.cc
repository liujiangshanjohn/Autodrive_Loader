#include "pid_config_manager.h"
#include <fstream>
#include <filesystem>

namespace cannode {

PIDConfigManager::PIDConfigManager(const std::string& config_file_path)
    : config_file_path_(config_file_path) {
    setDefaultConfig();
}

bool PIDConfigManager::loadConfig() {
    if (!configFileExists()) {
        // std::cerr << "[PIDConfigManager] 配置文件不存在: " << config_file_path_ << std::endl;
        // std::cerr << "[PIDConfigManager] 使用默认配置" << std::endl;
        return false;
    }
    
    try {
        YAML::Node config = YAML::LoadFile(config_file_path_);
        
        // 解析速度控制器配置
        if (config["speed_controller"]) {
            parsePIDConfig(config["speed_controller"], config_.speed_controller, false);
            // std::cout << "[PIDConfigManager] 速度控制器配置加载成功" << std::endl;
        } else {
            // std::cout << "[PIDConfigManager] 未找到速度控制器配置，使用默认值" << std::endl;
        }
        
        // 解析大臂控制器配置
        if (config["arm_controller"]) {
            parsePIDConfig(config["arm_controller"], config_.arm_controller, true);
            // std::cout << "[PIDConfigManager] 大臂控制器配置加载成功" << std::endl;
        } else {
            // std::cout << "[PIDConfigManager] 未找到大臂控制器配置，使用默认值" << std::endl;
        }
        
        // 解析铲斗控制器配置
        if (config["bucket_controller"]) {
            parsePIDConfig(config["bucket_controller"], config_.bucket_controller, true);
            // std::cout << "[PIDConfigManager] 铲斗控制器配置加载成功" << std::endl;
        } else {
            // std::cout << "[PIDConfigManager] 未找到铲斗控制器配置，使用默认值" << std::endl;
        }
        
        // 解析转向控制器配置
        if (config["steer_controller"]) {
            parsePIDConfig(config["steer_controller"], config_.steer_controller, true);
            // std::cout << "[PIDConfigManager] 转向控制器配置加载成功" << std::endl;
        } else {
            // std::cout << "[PIDConfigManager] 未找到转向控制器配置，使用默认值" << std::endl;
        }
        
        // 解析零点偏置配置
        if (config["zero_offsets"]) {
            parseZeroOffsetsConfig(config["zero_offsets"], config_.zero_offsets);
            // std::cout << "[PIDConfigManager] 零点偏置配置加载成功" << std::endl;
        } else {
            // std::cout << "[PIDConfigManager] 未找到零点偏置配置，使用默认值" << std::endl;
        }
        
        // std::cout << "[PIDConfigManager] 配置文件加载完成: " << config_file_path_ << std::endl;
        return true;
        
    } catch (const YAML::Exception& e) {
        // std::cerr << "[PIDConfigManager] YAML解析错误: " << e.what() << std::endl;
        // std::cerr << "[PIDConfigManager] 使用默认配置" << std::endl;
        return false;
    } catch (const std::exception& e) {
        // std::cerr << "[PIDConfigManager] 配置文件读取错误: " << e.what() << std::endl;
        // std::cerr << "[PIDConfigManager] 使用默认配置" << std::endl;
        return false;
    }
}

bool PIDConfigManager::reloadConfig() {
    // std::cout << "[PIDConfigManager] 重新加载配置文件..." << std::endl;
    return loadConfig();
}

bool PIDConfigManager::configFileExists() const {
    return std::filesystem::exists(config_file_path_);
}

void PIDConfigManager::setDefaultConfig() {
    // 设置默认的速度控制器参数
    config_.speed_controller.kp = 1.5;
    config_.speed_controller.ki = 0.2;
    config_.speed_controller.kd = 0.1;
    config_.speed_controller.max_output = 3.0;
    config_.speed_controller.min_output = -3.0;
    config_.speed_controller.max_integral = 5.0;
    config_.speed_controller.deadzone = 0.0;
    
    // 设置默认的大臂控制器参数
    config_.arm_controller.kp = 25.0;
    config_.arm_controller.ki = 0.8;
    config_.arm_controller.kd = 3.0;
    config_.arm_controller.max_output = 1500.0;
    config_.arm_controller.min_output = 0.0;
    config_.arm_controller.max_integral = 500.0;
    config_.arm_controller.deadzone = 0.5;
    
    // 设置默认的铲斗控制器参数
    config_.bucket_controller.kp = 25.0;
    config_.bucket_controller.ki = 0.8;
    config_.bucket_controller.kd = 3.0;
    config_.bucket_controller.max_output = 1500.0;
    config_.bucket_controller.min_output = 0.0;
    config_.bucket_controller.max_integral = 500.0;
    config_.bucket_controller.deadzone = 0.5;
    
    // 设置默认的转向控制器参数
    config_.steer_controller.kp = 20.0;
    config_.steer_controller.ki = 0.3;
    config_.steer_controller.kd = 0.0;
    config_.steer_controller.max_output = 500.0;
    config_.steer_controller.min_output = 0.0;
    config_.steer_controller.max_integral = 300.0;
    config_.steer_controller.deadzone = 0.1;
    
    // 设置默认的零点偏置参数
    config_.zero_offsets.boom_zero_deg = 0.0;
    config_.zero_offsets.bucket_zero_deg = 0.0;
    config_.zero_offsets.steer_zero_deg = 0.0;
}

void PIDConfigManager::parsePIDConfig(const YAML::Node& node, PIDConfig& pid_config, bool has_deadzone) {
    if (node["kp"]) {
        pid_config.kp = node["kp"].as<double>();
    }
    if (node["ki"]) {
        pid_config.ki = node["ki"].as<double>();
    }
    if (node["kd"]) {
        pid_config.kd = node["kd"].as<double>();
    }
    if (node["max_output"]) {
        pid_config.max_output = node["max_output"].as<double>();
    }
    if (node["min_output"]) {
        pid_config.min_output = node["min_output"].as<double>();
    }
    if (node["max_integral"]) {
        pid_config.max_integral = node["max_integral"].as<double>();
    }
    if (has_deadzone && node["deadzone"]) {
        pid_config.deadzone = node["deadzone"].as<double>();
    }
}

void PIDConfigManager::parseZeroOffsetsConfig(const YAML::Node& node, ZeroOffsetsConfig& zero_offsets) {
    if (node["boom_zero_deg"]) {
        zero_offsets.boom_zero_deg = node["boom_zero_deg"].as<double>();
    }
    if (node["bucket_zero_deg"]) {
        zero_offsets.bucket_zero_deg = node["bucket_zero_deg"].as<double>();
    }
    if (node["steer_zero_deg"]) {
        zero_offsets.steer_zero_deg = node["steer_zero_deg"].as<double>();
    }
}

} // namespace cannode
