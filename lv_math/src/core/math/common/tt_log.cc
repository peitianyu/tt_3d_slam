#include "tt_log.h"

#ifdef LOG_TO_FILE

LogFile::LogFile()
{
    m_file_name = LOG_FILE_PATH + std::string(__DATE__) + "_" + std::to_string(m_log_file_num++) + ".log";
    m_file.open(m_file_name, std::ios::out | std::ios::app);
}

LogFile::~LogFile() { m_file.close(); }

void LogFile::Flush() { m_file.flush(); }

std::string LogFile::GetFileName() { return m_file_name; }

std::size_t LogFile::GetFileSize() { return m_file.tellp(); }


uint LogFile::m_log_file_num = 0;


void LogFileManage::ManageFile()
{
    if (m_log_file == nullptr){
        m_log_file = new LogFile();
    }

    struct stat statbuf;
    stat(m_log_file->GetFileName().c_str(), &statbuf);

    if (statbuf.st_size > MAX_LOG_FILE_SIZE){
        delete m_log_file;
        m_log_file = new LogFile();

        RemoveFile();
    }

    if (m_log_file->GetFileSize() > MAX_LOG_CACHE_SIZE)
    {
        m_log_file->Flush();
    }
}

void LogFileManage::RemoveFile()
{
    // 删除超出最大文件数的文件
    std::string cmd = "ls " + std::string(LOG_FILE_PATH) + " -t | awk '{if(NR>10) print $0}'";
    FILE *fp = popen(cmd.c_str(), "r");
    if (fp == nullptr) return;

    char buf[1024];
    while (fgets(buf, sizeof(buf), fp) != nullptr){
        std::string file_name = std::string(LOG_FILE_PATH) + buf;
        file_name.erase(file_name.end() - 1);
        remove(file_name.c_str());
    }
}

LogFileManage::LogFileManage() : m_log_file(nullptr) {}
LogFileManage::~LogFileManage()
{
    if (m_log_file != nullptr){
        delete m_log_file;
        m_log_file = nullptr;
    }
}

#endif // LOG_TO_FILE