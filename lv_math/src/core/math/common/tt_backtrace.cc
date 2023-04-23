#include"tt_backtrace.h"
#include"tt_log.h"

void PrintTrace(int sig)
{
    void *array[10];
    size_t size;
    char **strings;
    size_t i;

    size = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    LOG_ERROR("------------------*** Error: signal %d ***------------------", sig) << std::endl;
    for (i = 0; i < size; i++)
        LOG_ERROR("[", i, "] ", strings[i]) << std::endl;

    free(strings);
    exit(1);
}


