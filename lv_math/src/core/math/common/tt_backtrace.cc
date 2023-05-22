#include"tt_backtrace.h"
#include <iostream>
#include <cxxabi.h>
#include <iomanip>

template <typename T>
static std::ostream &print_trace(std::ostream &os, const T &arg) { return os << arg;}

template <typename T, typename... Types>
static std::ostream &print_trace(std::ostream &os, const T &firstArg, const Types &...args)
{
    os << firstArg << " ";
    return print_trace(os, args...);
}

#define PRINT_TRACE(COLOR, ...) print_trace(std::cout, COLOR, __VA_ARGS__, "\033[0m")

void PrintTrace(int sig)
{
    void *array[50];

    size_t size = backtrace(array, 50);
    char **messages = backtrace_symbols(array, size);

    PRINT_TRACE("\033[31m", "------------------*** Error: signal " , sig, "***------------------") << std::endl;
    for (uint i = 1; i < size && messages != NULL; ++i)
    {
        char *mangled_name = 0, *offset_begin = 0, *offset_end = 0;
        for (char *p = messages[i]; *p; ++p){
            if (*p == '(') mangled_name = p;
            else if (*p == '+') offset_begin = p;
            else if (*p == ')') { offset_end = p;  break; }
        }

        if (mangled_name && offset_begin && offset_end && mangled_name < offset_begin){
            *mangled_name++ = '\0';
            *offset_begin++ = '\0';
            *offset_end++ = '\0';

            int status;
            char *real_name = abi::__cxa_demangle(mangled_name, 0, 0, &status);

            std::string str = "["+std::to_string(i)+"] " + offset_begin;
            if(str.size() < 15) str.append(15-str.size(), ' ');
            std::string str1 = messages[i];
            if(str1.size() < 35) str1.append(35-str1.size(), ' ');
            PRINT_TRACE("\033[33m", str, offset_end, "[", str1, "]:", status ? mangled_name : real_name) << std::endl;

            free(real_name);
        }
        else{ PRINT_TRACE("\033[33m", "[", i, "] ", messages[i]) << std::endl;}
    }

    free(messages);
    exit(1);
}


