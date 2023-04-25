#ifndef __COMMON_LOG_H__
#define __COMMON_LOG_H__

// #define LOG_TO_FILE
#ifdef LOG_TO_FILE

#include<fstream>
#include<sstream>
#include<sys/stat.h>
#include<mutex>

#define LOG_FILE_PATH "log/" // 修改日志文件存放路径

class LogFile
{
public:
    LogFile();
    ~LogFile();

    template <typename T>
    std::ofstream& WriteLog(const T &arg){
        m_file << arg;

        return m_file;
    }

    template <typename T, typename... Types>
    std::ofstream& WriteLog(const T &firstArg, const Types &...args){
        m_file << firstArg;
        return WriteLog(args...);
    }

    void Flush();
    std::string GetFileName();
    std::size_t GetFileSize();
private:
    static uint m_log_file_num;
    std::string m_file_name;
    std::ofstream m_file;
};


#define MAX_LOG_CACHE_SIZE 1024                         // 修改缓存日志大小
#define MAX_LOG_FILE_SIZE 1024 * 1024 * 10              // 修改单文件最大大小
#define MAX_LOG_FILE_NUM 10                             // 修改最大文件数


class LogFileManage
{
public:
    static LogFileManage *GetInstance(){
        static LogFileManage instance;
        return &instance;
    }

    template <typename T>
    std::ofstream& WriteLog(const T &arg)
    {
        m_mutex.lock();
        ManageFile();
        m_mutex.unlock();

        return m_log_file->WriteLog(arg);
    }

    template <typename T, typename... Types>
    std::ofstream& WriteLog(const T &firstArg, const Types &...args)
    {
        m_mutex.lock();
        ManageFile();
        m_log_file->WriteLog(firstArg);
        m_mutex.unlock();

        return WriteLog(args...);
    }

private:
    void ManageFile();

    void RemoveFile();

    LogFileManage();
    ~LogFileManage();

private:
    LogFile *m_log_file;
    std::mutex m_mutex;
};

#define LOG_FILE_CORE(...) LogFileManage::GetInstance()->WriteLog("[", __TIME__, "] ", __VA_ARGS__)

#define LOG_DEBUG(...) LOG_FILE_CORE("[DEBUG] [", __FUNCTION__, __LINE__, "]", __VA_ARGS__)
#define LOG_INFO(...) LOG_FILE_CORE(" [INFO] ", __VA_ARGS__)
#define LOG_WARN(...) LOG_FILE_CORE(" [WARN] [", __FUNCTION__, __LINE__, "]", __VA_ARGS__)
#define LOG_ERROR(...) LOG_FILE_CORE("[ERROR] [", __FUNCTION__, __LINE__, "]", __VA_ARGS__)

#else

#include<iostream>

template <typename T>
std::ostream &STD_COUT(std::ostream &os, const T &arg) { return os << arg;}
   
template <typename T, typename... Types>
std::ostream &STD_COUT(std::ostream &os, const T &firstArg, const Types &...args){ os << firstArg << " "; return STD_COUT(os, args...);}

#define LOG_DEBUG(...) STD_COUT(std::cout, "\033[33m[", __TIME__, "] [DEBUG] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_INFO(...) STD_COUT(std::cout, "\033[32m[", __TIME__, "]  [INFO] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_WARN(...) STD_COUT(std::cout, "\033[35m[", __TIME__, "]  [WARN] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")
#define LOG_ERROR(...) STD_COUT(std::cout, "\033[31m[", __TIME__, "] [ERROR] [", __FUNCTION__, __LINE__, "] : ", __VA_ARGS__, "\033[0m")

#endif // LOG_TO_FILE




#endif // __COMMON_LOG_H__