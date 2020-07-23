#ifndef LOGGING_H_
#define LOGGING_H_

#include <sstream>

enum LogSeverity
{
    kInfo,
    kWarning,
    kError,
    kFatal
};

class Logger : public std::ostringstream
{
  public:
    Logger(LogSeverity severity, const char* file_name, int line_number);
    Logger& stream() { return *this; }
    ~Logger();

  private:
    const LogSeverity severity_;
};

#define LOG(severity) LOG_##severity
#define LOG_IF(severity, condition)                                                                \
    if (condition)                                                                                 \
    LOG(severity)
#define CHECK(condition)                                                                           \
    if (condition)                                                                                 \
        0;                                                                                         \
    else                                                                                           \
        LOG(FATAL) << "Condition " << #condition << " failed: "

#define CHECK_EQ(val1, val2) CHECK_COMPARE(==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_COMPARE(!=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_COMPARE(>, val1, val2)
#define CHECK_GE(val1, val2) CHECK_COMPARE(>=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_COMPARE(<, val1, val2)
#define CHECK_LE(val1, val2) CHECK_COMPARE(<=, val1, val2)

#define CHECK_COMPARE(op, val1, val2)                                                              \
    if ((val1)op(val2))                                                                            \
        0;                                                                                         \
    else                                                                                           \
        LOG(FATAL) << "Comparison " << (#val1 " " #op " " #val2) << " failed " << '(' << val1      \
                   << " vs. " << val2 << "): "

#define LOG_INFO                                                                                   \
    if (false) else                                                                                \
    default:                                                                                       \
        Logger(LogSeverity::kInfo, __FILE__, __LINE__).stream()

#define LOG_ERROR                                                                                  \
    switch (0)                                                                                     \
    default:                                                                                       \
        Logger(LogSeverity::kError, __FILE__, __LINE__).stream()

#define LOG_WARNING                                                                                \
    switch (0)                                                                                     \
    default:                                                                                       \
        Logger(LogSeverity::kWarning, __FILE__, __LINE__).stream()

#define LOG_FATAL                                                                                  \
    switch (0)                                                                                     \
    default:                                                                                       \
        Logger(LogSeverity::kFatal, __FILE__, __LINE__).stream()


#endif  // LOGGING_H_

#ifdef LOGGING_H_IMPLEMENTATION

#include <cstdlib>
#include <ctime>
#include <iostream>

Logger::Logger(LogSeverity severity, const char* file_name, int line_number)
    : severity_(severity)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);

    struct tm local_timespec;
    localtime_s(&local_timespec, &ts.tv_sec);

    char buffer[64];
    strftime(buffer, sizeof(buffer), "%T", &local_timespec);

    static constexpr char kLevelAbbrev[] = {'I', 'W', 'E', 'F'};
    (*this) << kLevelAbbrev[severity] << ' ' << buffer << '.' << ts.tv_nsec / 1000 << "  "
            << file_name << ':' << line_number << "] ";
}

Logger::~Logger()
{
    std::cerr << str() << std::endl;
    if (severity_ == LogSeverity::kFatal) {
        std::abort();
    }
}

#endif  // LOGGING_H_IMPLEMENTATION
