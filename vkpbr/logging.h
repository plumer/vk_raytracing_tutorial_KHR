#ifndef LOGGING_H_
#define LOGGING_H_

#include <sstream>

enum class LogSeverity : int
{
    kInfo    = 0,
    kWarning = 1,
    kError   = 2,
    kFatal   = 3
};

class Logger : public std::ostringstream
{
  public:
    Logger(LogSeverity severity, const char* file_name, int line_number);
    Logger& stream() { return *this; }
    ~Logger() override;

  private:
    const LogSeverity severity_;
};

#define LOG(severity) LOG_##severity
#define LOG_IF(severity, condition) if (condition) LOG(severity)
#define CHECK(condition)                                                                           \
    if (condition)                                                                                 \
        0;                                                                                         \
    else                                                                                           \
        Logger(LogSeverity::kFatal, __FILE__, __LINE__).stream()                                   \
            << "Condition \"" << #condition << "\" failed: "

#define CHECK_EQ(val1, val2) CHECK_COMPARE(==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_COMPARE(!=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_COMPARE(>, val1, val2)
#define CHECK_GE(val1, val2) CHECK_COMPARE(>=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_COMPARE(<, val1, val2)
#define CHECK_LE(val1, val2) CHECK_COMPARE(<=, val1, val2)

#define CHECK_COMPARE(op, expr1, expr2)                                                            \
    if (decltype(expr1) val1 = expr1, val2 = expr2; (val1)op(val2))                                \
        0;                                                                                         \
    else                                                                                           \
        Logger(LogSeverity::kFatal, __FILE__, __LINE__).stream()                                   \
            << "Comparison " << (#expr1 " " #op " " #expr2) << " failed " << '(' << val1           \
            << " vs. " << val2 << "): "

#define LOG_INFO Logger(LogSeverity::kInfo, __FILE__, __LINE__).stream()

#define LOG_ERROR Logger(LogSeverity::kError, __FILE__, __LINE__).stream()

#define LOG_WARNING Logger(LogSeverity::kWarning, __FILE__, __LINE__).stream()

#define LOG_FATAL Logger(LogSeverity::kFatal, __FILE__, __LINE__).stream()

#ifdef _WIN32
#define LOG_0 LOG_ERROR
#endif


#endif  // LOGGING_H_
