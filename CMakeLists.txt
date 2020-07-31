project(wpimath)
cmake_minimum_required(VERSION 3.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(main_lib_dest wpimath/lib)
set(include wpimath/include)

INCLUDE(CPack)

# option(WITHOUT_JAVA "Do not include Java and JNI in the build" OFF)
option(WITH_TESTS "Build unit tests (requires Internet connection)" OFF)

# Set JNI sources.
file (GLOB_RECURSE wpimath_jni_src src/main/native/cpp/drake/jni/*.cpp)

# Set native sources.
file (GLOB_RECURSE wpimath_native_src src/main/native/cpp/*.cpp)

# Remove JNI from native sources.
list (REMOVE_ITEM wpimath_native_src ${wpimath_jni_src})

# Add library and set properties
add_library(wpimath ${wpimath_native_src})
set_target_properties(wpimath PROPERTIES DEBUG_POSTFIX "d")

# Specify include directories
target_include_directories(wpimath PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/main/native/include>
                            $<INSTALL_INTERFACE:${include_dest}/wpimath>)

set_property(TARGET wpimath PROPERTY folder "libraries")

install (TARGETS wpimath EXPORT wpimath DESTINATION "${main_lib_dest}")
install (DIRECTORY src/main/native/include DESTINATION "${include_dest}")

if (MSVC)
    set (wpimath_config_dir ${wpilib_dest})
else()
    set (wpimath_config_dir share/wpimath)
endif()

install(EXPORT wpimath DESTINATION ${wpimath_config_dir})

if (WITH_TESTS)
    enable_testing()
    add_subdirectory(googletest)
    include(GoogleTest)

    file (GLOB_RECURSE test_src src/test/native/cpp/*.cpp)
    add_executable(wpimath_test ${test_src})
    target_compile_definitions(wpimath_test PRIVATE -DGTEST_LINKED_AS_SHARED_LIBRARY)
    if (MSVC)
        target_compile_options(wpimath_test PRIVATE /wd4251 /wd4101)
    endif()

    add_test(NAME wpimath COMMAND wpimath_test)

    target_include_directories(wpimath_test PRIVATE src/test/native/include)
    target_link_libraries(wpimath_test wpimath gmock_main)
    if (NOT MSVC)
        target_compile_options(wpimath_test PRIVATE -Wno-error)
    else()
        target_compile_options(wpimath_test PRIVATE /WX-)
    endif()
endif()
