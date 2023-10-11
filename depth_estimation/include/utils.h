#ifndef __UTILS_H__
#define __UTILS_H__

#include <fmt/core.h>
#include <fmt/color.h>

#include <stdarg.h>

#define UTIL_BUF_SIZE 0x1000 // 4kb

namespace alert {

static inline void critic_runtime_error(const char* fmt, ...) {

    char buffer[UTIL_BUF_SIZE];
    memset(buffer, 0, UTIL_BUF_SIZE);

    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    std::string msg(buffer, rc);

    throw std::runtime_error(
        fmt::format("[{}] {}\n",
            fmt::format(fg(fmt::color::red), "CRITIC"),
            msg
        )
    );

};

static inline void warning_message(const char* fmt, ...) {

    char buffer[UTIL_BUF_SIZE];
    memset(buffer, 0, UTIL_BUF_SIZE);

    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    std::string msg(buffer, rc);

    fmt::print(
        fmt::format("[{}] {}\n",
            fmt::format(fg(fmt::color::purple), "CRITIC"),
            msg
        )
    );

};

static inline void info_message(const char* fmt, ...) {
    char buffer[UTIL_BUF_SIZE];
    memset(buffer, 0, UTIL_BUF_SIZE);

    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    std::string msg(buffer, rc);

    fmt::print(
        fmt::format("[{}] {}\n",
            fmt::format(fg(fmt::color::blue), "INFO"),
            msg
        )
    );

}

}

inline std::string methodName(const std::string& prettyFunction)
{
    size_t colons = prettyFunction.find("::");
    size_t begin = prettyFunction.substr(0,colons).rfind(" ") + 1;
    size_t end = prettyFunction.rfind("(") - begin;

    return prettyFunction.substr(begin,end) + "()";
}

#define __METHOD_NAME__ methodName(__PRETTY_FUNCTION__)

#endif
