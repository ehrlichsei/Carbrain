#ifndef COMMON_MACROS_H
#define COMMON_MACROS_H


// clang-format off
/*! Suppresses warnings in third party header files.
 * Works with
 *  - GCC (https://gcc.gnu.org/onlinedocs/gcc/Diagnostic-Pragmas.html)
 *  - clang (http://clang.llvm.org/docs/UsersManual.html#controlling-diagnostics-via-pragmas)
 *
 * For reasons GCC can not ignored -Wall, so all warnings have to be ignored
 * separatly.
 */
// clang-format on

#ifdef __clang__

#if __clang_major__ >= 6

// clang-format off
#define DISABLE_CLANG_6_WARNINGS \
  _Pragma("GCC diagnostic ignored \"-Wdelete-non-virtual-dtor\"") \
  _Pragma("GCC diagnostic ignored \"-Wsign-compare\"") \
  _Pragma("GCC diagnostic ignored \"-Wredundant-parens\"") \
  _Pragma("GCC diagnostic ignored \"-Woverloaded-virtual\"") \
  _Pragma("GCC diagnostic ignored \"-Wnewline-eof\"") \
  _Pragma("GCC diagnostic ignored \"-Wundef\"")
// clang-format on
#else

#define DISABLE_CLANG_6_WARNINGS /*nothing to do here */

#endif

#if __clang_major__ >= 5

// clang-format off
#define DISABLE_CLANG_5_WARNINGS \
  _Pragma("GCC diagnostic ignored \"-Wzero-as-null-pointer-constant\"") \
  _Pragma("GCC diagnostic ignored \"-Wcast-qual\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-template\"") \
  _Pragma("GCC diagnostic ignored \"-Wshadow-field\"") \
  _Pragma("GCC diagnostic ignored \"-Wimplicit-fallthrough\"")
// clang-format on
#else

#define DISABLE_CLANG_5_WARNINGS /*nothing to do here */

#endif

// clang-format off
#define THIRD_PARTY_HEADERS_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wunknown-pragmas\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-local-typedefs\"") \
  _Pragma("GCC diagnostic ignored \"-Wignored-qualifiers\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
  _Pragma("GCC diagnostic ignored \"-Wempty-body\"") \
  _Pragma("GCC diagnostic ignored \"-Wconversion\"") \
  _Pragma("GCC diagnostic ignored \"-Wpedantic\"") \
  _Pragma("GCC diagnostic ignored \"-Wold-style-cast\"") \
  _Pragma("GCC diagnostic ignored \"-Wnon-virtual-dtor\"") \
  _Pragma("GCC diagnostic ignored \"-Wextra-semi\"") \
  _Pragma("GCC diagnostic ignored \"-Wdocumentation\"") \
  _Pragma("GCC diagnostic ignored \"-Wpedantic\"") \
  _Pragma("GCC diagnostic ignored \"-Wpadded\"") \
  _Pragma("GCC diagnostic ignored \"-Wdouble-promotion\"") \
  _Pragma("GCC diagnostic ignored \"-Wweak-vtables\"") \
  _Pragma("GCC diagnostic ignored \"-Wundefined-func-template\"") \
  _Pragma("GCC diagnostic ignored \"-Wcomma\"") \
  _Pragma("GCC diagnostic ignored \"-Wexit-time-destructors\"") \
  _Pragma("GCC diagnostic ignored \"-Wreserved-id-macro\"") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated\"") \
  _Pragma("GCC diagnostic ignored \"-Wdocumentation-unknown-command\"") \
  _Pragma("GCC diagnostic ignored \"-Wmissing-noreturn\"") \
  _Pragma("GCC diagnostic ignored \"-Wglobal-constructors\"") \
  _Pragma("GCC diagnostic ignored \"-Wundefined-reinterpret-cast\"") \
  _Pragma("GCC diagnostic ignored \"-Wused-but-marked-unused\"") \
  _Pragma("GCC diagnostic ignored \"-Wcast-align\"") \
  _Pragma("GCC diagnostic ignored \"-Wfloat-equal\"") \
  _Pragma("GCC diagnostic ignored \"-Wswitch-enum\"") \
  _Pragma("GCC diagnostic ignored \"-Wshift-sign-overflow\"") \
  _Pragma("GCC diagnostic ignored \"-Wformat-nonliteral\"") \
  _Pragma("GCC diagnostic ignored \"-Wcovered-switch-default\"") \
  _Pragma("GCC diagnostic ignored \"-Wmismatched-tags\"") \
  _Pragma("GCC diagnostic ignored \"-Wzero-as-null-pointer-constant\"") \
  _Pragma("GCC diagnostic ignored \"-Wreturn-type-c-linkage\"") \
  _Pragma("GCC diagnostic ignored \"-Wmissing-variable-declarations\"") \
  _Pragma("GCC diagnostic ignored \"-Wused-but-marked-unused\"") \
  DISABLE_CLANG_5_WARNINGS \
  DISABLE_CLANG_6_WARNINGS
// clang-format on
#else


#if __GNUC__ >= 7

// clang-format off
#define DISABLE_GCC_7_WARNINGS \
  _Pragma("GCC diagnostic ignored \"-Wregister\"") \
  _Pragma("GCC diagnostic ignored \"-Wshadow=local\"") \
  _Pragma("GCC diagnostic ignored \"-Wshadow=compatible-local\"") \
  _Pragma("GCC diagnostic ignored \"-Wsign-compare\"")
// clang-format on
#else

#define DISABLE_GCC_7_WARNINGS /*nothing to do here */

#endif

// clang-format off
#define THIRD_PARTY_HEADERS_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wunknown-pragmas\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-local-typedefs\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-but-set-parameter\"") \
  _Pragma("GCC diagnostic ignored \"-Wuseless-cast\"") \
  _Pragma("GCC diagnostic ignored \"-Wignored-qualifiers\"") \
  _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
  _Pragma("GCC diagnostic ignored \"-Wempty-body\"") \
  _Pragma("GCC diagnostic ignored \"-Wconversion\"") \
  _Pragma("GCC diagnostic ignored \"-Wformat=\"") \
  _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"") \
  _Pragma("GCC diagnostic ignored \"-Wold-style-cast\"") \
  _Pragma("GCC diagnostic ignored \"-Wnon-virtual-dtor\"") \
  _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"") \
  _Pragma("GCC diagnostic ignored \"-Wzero-as-null-pointer-constant\"") \
  _Pragma("GCC diagnostic ignored \"-Wcast-qual\"") \
  _Pragma("GCC diagnostic ignored \"-Wlogical-op\"") \
  _Pragma("GCC diagnostic ignored \"-Wredundant-decls\"") \
  _Pragma("GCC diagnostic ignored \"-Wconversion\"") \
  _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"") \
  _Pragma("GCC diagnostic ignored \"-Wpedantic\"") \
  DISABLE_GCC_7_WARNINGS
// clang-format on
#endif

// clang-format off
#define THIRD_PARTY_HEADERS_END \
  _Pragma("GCC diagnostic pop")
// clang-format on

#ifdef __clang__

#define DISABLE_SUGGEST_OVERRIDE_WARNING

#else

/*!
 * -Wsuggest-override is raised by gtest by default because of its internals and
 * can not be disabled by warning guards because of it occurse in  macros.
 * Therefore we need to disable this warning for the hole file.
 */
#define DISABLE_SUGGEST_OVERRIDE_WARNING \
  _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"")

#endif

#ifdef __clang__

/*!
 * -Wused-but-marked-unused is raised by gtest's macro EXPECT_DEATH and
 * can not be disabled by warning guards because of it occurse in macros.
 */
#define DISABLE_USED_BUT_MARKED_UNUSED_WARNING \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wused-but-marked-unused\"")

#define ENABLE_USED_BUT_MARKED_UNUSED_WARNING \
  _Pragma("GCC diagnostic pop")

#else

#define DISABLE_USED_BUT_MARKED_UNUSED_WARNING
#define ENABLE_USED_BUT_MARKED_UNUSED_WARNING

#endif



// clang-format off
/*! Suppress -Wunused-parameter for a specific parameter.
 * Works with
 *  - GCC (https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html#Common-Function-Attributes)
 *  - clang (http://clang.llvm.org/docs/AttributeReference.html#maybe-unused-unused-gnu-unused)
 */
// clang-format on
#define UNUSED __attribute__((__unused__))

#endif
