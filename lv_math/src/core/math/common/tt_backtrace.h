#ifndef __COMMON_BACKTRACE_H__
#define __COMMON_BACKTRACE_H__

#include <execinfo.h>
#include <signal.h>

void PrintTrace(int sig);

#define REGISTER_SEGFAULT_HANDLER signal(SIGSEGV, PrintTrace);

#endif // __COMMON_BACKTRACE_H__