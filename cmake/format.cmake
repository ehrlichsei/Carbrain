find_program(CLANG_FORMAT
             NAMES clang-format clang-format-3.6 clang-format-3.5 clang-format-3.4
             PATHS ENV PATH)

if(NOT EXISTS ${CLANG_FORMAT})
    #message(FATAL_ERROR "clang-format not found. please install it.")
    message(ERROR "clang-format not found. please install it.")
else()
    message("clang-format found")
    message(${CLANG_FORMAT})
endif()
