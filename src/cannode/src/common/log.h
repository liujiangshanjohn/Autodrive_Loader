#pragma once

#include <stdio.h>
#include <unistd.h>

#include <string>
#include <memory>
#include <functional>

// 如果不想用header only模式注释掉下面两个宏定义，并在cmakelists中添加链接spdlog库fmt库
#define SPDLOG_HEADER_ONLY
#define FMT_HEADER_ONLY

#include <spdlog/spdlog.h>

#if SPDLOG_VERSION < 10902
#error "spdlog version >= 1.9.2 required"
#endif

#include <spdlog/logger.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/fmt/bin_to_hex.h>
#include <spdlog/fmt/fmt.h>

// 日志文件后缀
constexpr const char* kLogFileSuffix = ".log";
constexpr const size_t kLogFileSuffixLen = 4;
// 年月日 时分秒.毫秒 时区 模块名 日志等级 线程号 文件名 函数名 行数 # 用户自定义日志消息
constexpr const char* kDefaultPattern = "%D %T.%f %z %n %L %t %s %! %# # %v"; 
// 日志队列最大长度
constexpr const size_t kDefaultThreadQueueSize = 1024 * 32; // 32k
// 日志线程数
constexpr const size_t kDefaultThreadNum = 1;
// 日志文件默认路径
constexpr const char* kDefaultLogRootPath = ".";
// 日志文件默认大小
constexpr const size_t kDefaultRotatingFileMaxSize = 1024 * 1024 * 10; /*10M*/
// 日志文件默认个数
constexpr const size_t kDefaultRotatingFileNum = 1; //2个文件，1个备份

#define LOG_LEVEL_MAX 6
#define LOG_LEVEL_MIN 0

// @brief 异步日志类，队列最大32k，超出队列丢弃，按照文件大小滚动记录
class Logger final {
public:

    enum Level : uint8_t {
        OFF = LOG_LEVEL_MAX - spdlog::level::off,
        FATEL = LOG_LEVEL_MAX - spdlog::level::critical,
        ERROR = LOG_LEVEL_MAX - spdlog::level::err,
        WARN = LOG_LEVEL_MAX - spdlog::level::warn,
        INFO = LOG_LEVEL_MAX - spdlog::level::info,
        DEBUG = LOG_LEVEL_MAX - spdlog::level::debug,
        VERBOSE = LOG_LEVEL_MAX - spdlog::level::trace
    };

    enum Mode : uint8_t {
        kOff = 0x00,
        kConsole = 0x01,
        kRotatingFile = 0x02,
    };

    inline static spdlog::level::level_enum ToLevel(Level level) {
        return static_cast<spdlog::level::level_enum>(LOG_LEVEL_MAX - level);
    }

    inline static Level ToLevel(spdlog::level::level_enum level) {
        return static_cast<Level>(LOG_LEVEL_MAX - level);
    }

    // 单例
    inline static Logger& GetInstance() { static Logger t; return t; }

    // @brief 初始化日志
    // @param logger_name 日志名称，日志文件命名格式为<file_path>/<logger_name>.log，logger_name建议为模块名称（PROJECT_NAME）
    // @param mode 日志输出模式，0表示不输出，1表示控制台输出，2表示文件输出，3表示控制台和文件都输出
    // @param log_level 日志打印级别，默认info
    // @param flush_level 日志刷新级别，默认error
    void Init(const char* logger_name, uint8_t mode = kConsole, Level log_level = INFO, Level flush_level = ERROR) {

        spdlog::init_thread_pool(kDefaultThreadQueueSize, kDefaultThreadNum);
        spdlog::set_pattern(kDefaultPattern);

        std::vector<spdlog::sink_ptr> sinks;
        if(mode & kRotatingFile) {
            if(this->file_path_.empty()) { 
                this->file_path_ = kDefaultLogRootPath; 
            }
            // 本地文件 去除path最后的/
            if(this->file_path_.back() == '/') 
                this->file_path_.pop_back();

            // 如果文件路径是以.log为结尾，则视为文件路径，否则视为目录
            // 判断是否是.log结尾
            if(this->file_path_.length() < kLogFileSuffixLen
                || this->file_path_.rfind(kLogFileSuffix) != this->file_path_.length() - kLogFileSuffixLen) {
                    this->file_path_ = fmt::format("{}/{}{}", this->file_path_, logger_name, kLogFileSuffix);
                } 
            
            if(this->log_file_num_ == 0) {
                sinks.emplace_back(
                    std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                        this->file_path_));
            } else {
                sinks.emplace_back(
                    std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                        this->file_path_, this->log_file_size_, this->log_file_num_));
            }
        }
            
        if(mode & kConsole) {
            sinks.emplace_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
        }
        
        if(sinks.empty()) return;

        this->logger_ = std::make_shared<spdlog::async_logger>(
            logger_name, sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);

        this->log_level_ = log_level;
        this->flush_level_ = flush_level;
        this->logger_->flush_on(this->ToLevel(this->flush_level_));
        this->logger_->set_level(this->ToLevel(this->log_level_));
        this->logger_->set_error_handler(this->err_handler_);
        this->logger_->set_pattern(this->pattern_);
        spdlog::set_default_logger(this->logger_);

        this->is_running_ = true;
        SPDLOG_LOGGER_CALL(this->logger_, spdlog::level::info,
            "Logger initialized: version={}, mode={}, path={}, log_level={}, flush_level={}", 
            SPDLOG_VERSION, (int)mode, this->file_path_, (int)this->log_level_, (int)this->flush_level_);
        return;
    }

    inline void Exit() { 
        this->logger_->flush();
        this->logger_.reset();
        this->is_running_ = false; 
        spdlog::shutdown(); 
    }

    // 以下接口需要在init前调用
    // 重设日志格式
    inline void SetPattern(const std::string& pattern) { this->pattern_ = pattern; }
    // 设置错误输出回调
    inline void SetErrHandler(std::function<void(const std::string&)> handler) { this->err_handler_ = handler; }
    // 设置滚动文件大小，单位MB
    inline void SetLogFileSize(size_t size) { this->log_file_size_ = size * 1024 * 1024; } 
    // 设置滚动文件个数，0表示不滚动单个文件记录（同时文件大小参数失效）
    inline void SetLogFileNum(size_t num) { this->log_file_num_ = num; }
    // 设置日志文件路径，可以是目录，也可以是具体文件，当以.log结尾时表示具体文件，默认是当前目录
    inline void SetLogFilePath(const std::string& path) { this->file_path_ = path; }

    // 以下接口可以在init后调用，随时重设日志级别，立即生效
    inline void SetPrintLogLevel(Level level) { this->log_level_ = level; spdlog::set_level(this->ToLevel(level)); }
    inline void SetFlushLogLevel(Level level) { this->flush_level_ = level; spdlog::flush_on(this->ToLevel(level)); }

    inline bool IsRunning() const { return this->is_running_; }
private:
    // 默认打印info级别，遇到error级别刷新
    Logger() : 
        is_running_(false), 
        logger_(), 
        err_handler_(), 
        pattern_(kDefaultPattern), 
        file_path_(kDefaultLogRootPath), 
        log_file_size_(kDefaultRotatingFileMaxSize), 
        log_file_num_(kDefaultRotatingFileNum),
        log_level_(INFO),
        flush_level_(ERROR) {}

    Logger(const Logger&) = delete;
    Logger& operator = (const Logger&) = delete;

private:
    std::atomic_bool is_running_;
    std::shared_ptr<spdlog::logger> logger_;
    std::function<void(const std::string&)> err_handler_;
    std::string pattern_;
    std::string file_path_;
    size_t log_file_size_;
    size_t log_file_num_;
    Level log_level_;
    Level flush_level_;
};

/// @brief 日志打印宏
#define VLOG(level, format, ...) { \
    auto _p_logger = Logger::GetInstance().IsRunning()? spdlog::default_logger() : nullptr; \
    if(_p_logger) { SPDLOG_LOGGER_CALL(_p_logger, Logger::ToLevel(level), format, ##__VA_ARGS__); } \
    else { fmt::print(format "\n", ##__VA_ARGS__); } }

#define LOG_F(format, ...) VLOG(Logger::FATEL, format, ##__VA_ARGS__)
#define LOG_E(format, ...) VLOG(Logger::ERROR, format, ##__VA_ARGS__)
#define LOG_W(format, ...) VLOG(Logger::WARN, format, ##__VA_ARGS__)
#define LOG_I(format, ...) VLOG(Logger::INFO, format, ##__VA_ARGS__)
#define LOG_D(format, ...) VLOG(Logger::DEBUG, format, ##__VA_ARGS__)
#define LOG_V(format, ...) VLOG(Logger::VERBOSE, format, ##__VA_ARGS__)

// 使用方式-可执行文件
// #include <log.h>
// Logger::GetInstance().SetLogFilePath("./"); // 设置日志文件路径，可以是目录，也可以是具体文件
// Logger::GetInstance().SetLogFileSize(10); // 设置日志文件大小，单位MB
// Logger::GetInstance().SetLogFileNum(5); // 设置日志文件个数，0表示不滚动单个文件记录
// Logger::GetInstance().Init("task_mng", Logger::kRotatingFile | Logger::kConsole, Logger::WARN, Logger::WARN); // 初始化日志
// LOG_I("hello {}", "world"); // 打印日志
// Logger::GetInstance().SetPrintLogLevel(Logger::DEBUG); // 重新设置打印日志级别
// Logger::GetInstance().SetFlushLogLevel(Logger::ERROR); // 重新设置刷新日志级