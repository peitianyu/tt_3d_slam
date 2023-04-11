#ifndef __COMMON_LOG_H__
#define __COMMON_LOG_H__

#include<iostream>

template <typename T>
std::ostream &STD_COUT(std::ostream &os, const T &arg)
{
    return os << arg;
}

template <typename T, typename... Types>
std::ostream &STD_COUT(std::ostream &os, const T &firstArg, const Types &...args)
{
    os << firstArg << " ";
    return STD_COUT(os, args...);
}

#define LOG_INFO(...) STD_COUT(std::cout, __VA_ARGS__)
#define LOG_DEBUG(...) STD_COUT(std::cout, "\033[33m[", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_WARN(...) STD_COUT(std::cout, "\033[35m[", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_ERROR(...) STD_COUT(std::cout, "\033[31m[", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")


#endif // __COMMON_LOG_H__