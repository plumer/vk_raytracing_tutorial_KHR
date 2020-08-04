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
    if (false) 0; else Logger(LogSeverity::kInfo, __FILE__, __LINE__).stream()

#define LOG_ERROR                                                                                  \
    if (false) 0; else Logger(LogSeverity::kError, __FILE__, __LINE__).stream()

#define LOG_WARNING                                                                                \
    if (false) 0; else Logger(LogSeverity::kWarning, __FILE__, __LINE__).stream()

#define LOG_FATAL                                                                                  \
    if (false) 0; else Logger(LogSeverity::kFatal, __FILE__, __LINE__).stream()


#endif  // LOGGING_H_


