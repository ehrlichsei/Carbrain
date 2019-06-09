#pragma once

#include <assert.h>

#ifdef NDEBUG

#define assert_fail(MSG) (__ASSERT_VOID_CAST(0))

#else

#define assert_fail(MSG) \
  (__assert_fail(MSG, __FILE__, __LINE__, __ASSERT_FUNCTION))

#endif
