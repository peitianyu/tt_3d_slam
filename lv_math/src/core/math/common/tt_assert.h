#ifndef __COMMON_ASSERT_H__
#define __COMMON_ASSERT_H__

#include "tt_log.h"

#define tt_assert(expr)                                                                         \
    if (!(expr)) {                                                                              \
        LOG_ERROR("assert failed: ", #expr, __FILE__, __FUNCTION__, __LINE__) << std::endl;     \
        exit(1);                                                                                \
    }


#endif // __COMMON_ASSERT_H__