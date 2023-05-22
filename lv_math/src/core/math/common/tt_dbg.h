#ifndef __DBG_H__
#define __DBG_H__

#include <iostream>
#include <sstream>
#include <cxxabi.h>

#define COLOR_GREEN "\033[32m"
#define COLOR_BLUE "\033[34m"
#define COLOR_WHITE "\033[37m"
#define COLOR_GRAY "\033[90m"

class DebugOutput
{
public:
    DebugOutput(const std::string& file, int32_t line, const std::string& func, const std::string& expr)
    {
        std::ostringstream oss;
        oss << COLOR_GRAY << "[" << file << ":" << line << " (" << func << ")] " << COLOR_BLUE << expr << COLOR_WHITE << " = ";
        prefix_ = oss.str();
    }

    template<typename T>
    void print(const T& value){
        std::cout << prefix_ << value << " (" << COLOR_GREEN << type_name(value) << COLOR_WHITE << ")" << "\033[0m" << std::endl;
    }

    template<typename T>
    void print_vector(const std::vector<T>& value){
        std::cout << COLOR_GRAY << prefix_ << "{";
        for (size_t i = 0; i < value.size(); ++i){
            if (i > 0) std::cout << ", ";
            std::cout << value[i];
        }
        std::cout << "} (" << COLOR_GREEN << type_name(value) << COLOR_WHITE << ")" << "\033[0m" << std::endl;
    }
private:
    template <typename T>
    std::string type_name(const T& value)
    {
        int status;
        return abi::__cxa_demangle(typeid(value).name(), 0, 0, &status);
    }
private:
    std::string prefix_;
};

#define dbg(...) DebugOutput(__FILE__, __LINE__, __FUNCTION__, #__VA_ARGS__).print(__VA_ARGS__)
#define dbg_vector(...) DebugOutput(__FILE__, __LINE__, __FUNCTION__, #__VA_ARGS__).print_vector(__VA_ARGS__) 

#endif // __DBG_H__