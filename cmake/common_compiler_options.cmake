set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BASE_LINE_FLAGS "-Wall -fstack-protector -Wextra -Wnon-virtual-dtor -Werror=old-style-cast -Werror=return-type -Werror=reorder -Wmissing-braces -Wmissing-field-initializers -fPIC")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(BASE_LINE_FLAGS "${BASE_LINE_FLAGS} -Wsuggest-override -Wuseless-cast -Wcast-qual -Wlogical-op -Wredundant-decls")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    #TODO remove -Wno-except-type as soon as we use C++17
    set(BASE_LINE_FLAGS "${BASE_LINE_FLAGS} -Wno-noexcept-type -Wshadow=local -Wduplicated-cond")
  endif()
  if(NOT "$ENV{HOSTNAME}" STREQUAL "kitcarnuc")
    # This makes all warnings into errors (with two exceptions). This is NOT done on the car, since it would be too annoying.
    # unused-variable and unused-but-set-variable are excluded because these warnings come up frequently during debugging and
    # would be distrubing.
    set(BASE_LINE_FLAGS "${BASE_LINE_FLAGS} -Werror -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
  endif()
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(BASE_LINE_FLAGS "${BASE_LINE_FLAGS} -Weverything -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-c99-extensions -Wno-shadow -Wno-date-time -Wno-missing-prototypes -Wno-exit-time-destructors -Wno-global-constructors -Wno-conversion -Wno-float-conversion -Wno-double-promotion -Wno-sign-conversion -Wno-float-equal -Wno-padded -Wno-weak-vtables -Wno-abstract-vbase-init -Wno-potentially-evaluated-expression -Wno-zero-as-null-pointer-constant -Wno-undefined-func-template -Wno-covered-switch-default -Werror=inconsistent-missing-override -Werror=inconsistent-missing-destructor-override -Werror=header-hygiene -Werror=shadow-field")

# -Wcovered-switch-default is disabled because clang-tidy can detect this too and can be configure more fine-grained.
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${BASE_LINE_FLAGS} -g -Og -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_RELEASE "${BASE_LINE_FLAGS} -O3 -fdevirtualize-at-ltrans -fipa-pta -fuse-linker-plugin -fno-fat-lto-objects -flto -Wl,-flto -DNDEBUG -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} -g -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CONFIGURATION_TYPES ${CMAKE_CONFIGURATION_TYPES} Warn Stfu Unoptimized)
set(CMAKE_CXX_FLAGS_WARN "${BASE_LINE_FLAGS}  -Wconversion")
set(CMAKE_CXX_FLAGS_STFU "${CMAKE_CXX_FLAGS_RELEASE} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_NONE")
set(CMAKE_CXX_FLAGS_COVERAGE "${BASE_LINE_FLAGS} -fprofile-arcs -ftest-coverage -fPIC -Og -fsanitize=address -g")
set(CMAKE_CXX_FLAGS_UNOPTIMIZED "${BASE_LINE_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(CMAKE_AR "gcc-ar")
  set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> qcs <TARGET> <LINK_FLAGS> <OBJECTS>")
  set(CMAKE_CXX_ARCHIVE_FINISH true)
endif()
