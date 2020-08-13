#define LOGGING_H_IMPLEMENTATION
#include "logging.h"
#ifdef LOGGING_H_IMPLEMENTATION

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <cstdio>

Logger::Logger(LogSeverity severity, const char* file_name, int line_number)
    : severity_(severity) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);

    struct tm local_timespec;
    localtime_s(&local_timespec, &ts.tv_sec);

    // Prints local time into the buffer, precise to microseconds.
    char buffer[64] = {0};
    int  tmp = strftime(buffer, sizeof(buffer), "%T", &local_timespec);
    sprintf(buffer + tmp, ".%06d", ts.tv_nsec / 1000);
    const char* last_file_name = file_name + std::strlen(file_name) - 1;
    while (last_file_name >= file_name) {
        if (*last_file_name == '/' || *last_file_name == '\\')
            break;
        else
            last_file_name--;
    }
    ++last_file_name;

    static constexpr char kLevelAbbrev[] = {'I', 'W', 'E', 'F'};
    (*this) << kLevelAbbrev[severity] << ' ' << buffer << "  " << last_file_name << ':'
            << line_number << "] ";
}

Logger::~Logger() {
    std::cerr << str() << std::endl;
    if (severity_ == LogSeverity::kFatal) {
        std::abort();
    }
}

#endif  // LOGGING_H_IMPLEMENTATION