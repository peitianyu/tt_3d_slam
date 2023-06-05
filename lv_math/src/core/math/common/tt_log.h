#ifndef __LOG_H__
#define __LOG_H__

#include<iostream>
template <typename T>
std::ostream &STD_COUT(std::ostream &os, const T &arg) { return os << arg;}
   
template <typename T, typename... Types>
std::ostream &STD_COUT(std::ostream &os, const T &firstArg, const Types &...args){ os << firstArg << " "; return STD_COUT(os, args...);}

#define LOG(...)  STD_COUT(std::cout, __VA_ARGS__)
#define LOG_FILE(ofs, ...)  STD_COUT(ofs, __VA_ARGS__)

#define LOG_DEBUG(...) STD_COUT(std::cout, "\033[33m[", __TIME__, "] [DEBUG] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_INFO(...) STD_COUT(std::cout, "\033[32m[", __TIME__, "]  [INFO] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_WARN(...) STD_COUT(std::cout, "\033[35m[", __TIME__, "]  [WARN] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_ERROR(...) STD_COUT(std::cout, "\033[31m[", __TIME__, "] [ERROR] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")

#define LOG_DEBUG_FILE(ofs, ...) STD_COUT(ofs, __TIME__, "] [DEBUG] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__)
#define LOG_INFO_FILE(ofs, ...) STD_COUT(ofs, __TIME__, "]  [INFO] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__)
#define LOG_WARN_FILE(ofs, ...) STD_COUT(ofs, __TIME__, "]  [WARN] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__)
#define LOG_ERROR_FILE(ofs, ...) STD_COUT(ofs, __TIME__, "] [ERROR] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__)

#endif // __LOG_H__