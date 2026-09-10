#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace cannode {

/**
 * @brief PID控制器配置参数结构体
 */
struct PIDConfig {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_output = 0.0;
    double min_output = 0.0;
    double max_integral = 0.0;
    double deadzone = 0.0;  // 仅用于角度控制器
};

/**
 * @brief 零点偏置配置结构体
 */
struct ZeroOffsetsConfig {
    double boom_zero_deg = 0.0;
    double bucket_zero_deg = 0.0;
    double steer_zero_deg = 0.0;
};

/**
 * @brief 完整的PID配置结构体
 */
struct PIDControllersConfig {
    PIDConfig speed_controller;
    PIDConfig arm_controller;
    PIDConfig bucket_controller;
    PIDConfig steer_controller;
    ZeroOffsetsConfig zero_offsets;
};

/**
 * @brief PID配置管理器
 * 
 * 负责从YAML配置文件中读取PID参数，并提供默认值
 */
class PIDConfigManager {
public:
    /**
     * @brief 构造函数
     * @param config_file_path 配置文件路径
     */
    explicit PIDConfigManager(const std::string& config_file_path);
    
    /**
     * @brief 析构函数
     */
    ~PIDConfigManager() = default;
    
    /**
     * @brief 加载配置文件
     * @return true 成功，false 失败
     */
    bool loadConfig();
    
    /**
     * @brief 重新加载配置文件（用于运行时更新）
     * @return true 成功，false 失败
     */
    bool reloadConfig();
    
    /**
     * @brief 获取完整的PID配置
     * @return PIDControllersConfig 配置结构体
     */
    const PIDControllersConfig& getConfig() const { return config_; }
    
    /**
     * @brief 获取速度控制器配置
     * @return PIDConfig 速度控制器配置
     */
    const PIDConfig& getSpeedControllerConfig() const { return config_.speed_controller; }
    
    /**
     * @brief 获取大臂控制器配置
     * @return PIDConfig 大臂控制器配置
     */
    const PIDConfig& getArmControllerConfig() const { return config_.arm_controller; }
    
    /**
     * @brief 获取铲斗控制器配置
     * @return PIDConfig 铲斗控制器配置
     */
    const PIDConfig& getBucketControllerConfig() const { return config_.bucket_controller; }
    
    /**
     * @brief 获取转向控制器配置
     * @return PIDConfig 转向控制器配置
     */
    const PIDConfig& getSteerControllerConfig() const { return config_.steer_controller; }
    
    /**
     * @brief 获取零点偏置配置
     * @return ZeroOffsetsConfig 零点偏置配置
     */
    const ZeroOffsetsConfig& getZeroOffsetsConfig() const { return config_.zero_offsets; }
    
    /**
     * @brief 检查配置文件是否存在
     * @return true 存在，false 不存在
     */
    bool configFileExists() const;
    
    /**
     * @brief 获取配置文件路径
     * @return std::string 配置文件路径
     */
    const std::string& getConfigFilePath() const { return config_file_path_; }

private:
    std::string config_file_path_;
    PIDControllersConfig config_;
    
    /**
     * @brief 设置默认配置值
     */
    void setDefaultConfig();
    
    /**
     * @brief 从YAML节点解析PID配置
     * @param node YAML节点
     * @param pid_config 输出的PID配置
     * @param has_deadzone 是否包含死区参数
     */
    void parsePIDConfig(const YAML::Node& node, PIDConfig& pid_config, bool has_deadzone = false);
    
    /**
     * @brief 从YAML节点解析零点偏置配置
     * @param node YAML节点
     * @param zero_offsets 输出的零点偏置配置
     */
    void parseZeroOffsetsConfig(const YAML::Node& node, ZeroOffsetsConfig& zero_offsets);
};

} // namespace cannode
