#ifndef __COMMON_ASSERT_H__
#define __COMMON_ASSERT_H__



#define tt_assert(expr)                                                                         \
    if (!(expr)) {                                                                              \
        printf("Assert failed: %s, %s, %s, %d\n", #expr, __FILE__, __FUNCTION__, __LINE__);     \
        exit(1);                                                                                \
    }


#endif // __COMMON_ASSERT_H__