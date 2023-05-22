#include"common/tt_test.h"
#include "common/tt_backtrace.h"

int main()
{
    REGISTER_SEGFAULT_HANDLER
    
    return RunAllTests();
}