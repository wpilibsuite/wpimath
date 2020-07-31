project(wpimath)
cmake_minimum_required(VERSION 3.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(wpimath_dest wpimath)
set(main_lib_dest wpimath/lib)
set(include_dest wpimath/include)
set(java_lib_dest wpimath/java)
set(jni_lib_dest wpimath/jni)

INCLUDE(CPack)

option(WITHOUT_JAVA "Do not include Java and JNI in the build" OFF)
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

if (MSVC)
    target_compile_options(wpimath PRIVATE /bigobj)
endif()

set_property(TARGET wpimath PROPERTY folder "libraries")

install (TARGETS wpimath EXPORT wpimath DESTINATION "${main_lib_dest}")
install (DIRECTORY src/main/native/include DESTINATION "${include_dest}")

if (MSVC)
    set (wpimath_config_dir ${wpimath_dest})
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
    if (NOT MSVC)
        target_compile_definitions(wpimath_test PRIVATE -DGTEST_LINKED_AS_SHARED_LIBRARY)
    else()
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

if (NOT WITHOUT_JAVA) 
    find_package (Java REQUIRED)
    find_package (JNI REQUIRED)
    include(UseJava)
    set(CMAKE_JAVA_COMPILE_FLAGS "-Xlint:unchecked")

    if (NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/ejml/ejml-simple-0.38.jar")
        set(BASE_URL "https://search.maven.org/remotecontent?filepath=")
        set(JAR_ROOT "${CMAKE_CURRENT_BINARY_DIR}/ejml")

        message(STATUS "Downloading EJML jarfiles...")

        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-cdense/0.38/ejml-cdense-0.38.jar"
            "${JAR_ROOT}/ejml-cdense-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-core/0.38/ejml-core-0.38.jar"
            "${JAR_ROOT}/ejml-core-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-ddense/0.38/ejml-ddense-0.38.jar"
            "${JAR_ROOT}/ejml-ddense-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-dsparse/0.38/ejml-dsparse-0.38.jar"
            "${JAR_ROOT}/ejml-dsparse-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-fdense/0.38/ejml-fdense-0.38.jar"
            "${JAR_ROOT}/ejml-fdense-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-simple/0.38/ejml-simple-0.38.jar"
            "${JAR_ROOT}/ejml-simple-0.38.jar")
        file(DOWNLOAD "${BASE_URL}org/ejml/ejml-zdense/0.38/ejml-zdense-0.38.jar"
            "${JAR_ROOT}/ejml-zdense-0.38.jar")

        message(STATUS "All files downloaded.")
    endif()

    file(GLOB EJML_JARS ${CMAKE_CURRENT_BINARY_DIR}/ejml/*.jar)
    set(CMAKE_JAVA_INCLUDE_PATH wpimath.jar ${EJML_JARS})

    execute_process(COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/generate_numbers.py ${CMAKE_CURRENT_BINARY_DIR} RESULT_VARIABLE generateResult)
    if(NOT (generateResult EQUAL "0"))
        # Try python
        execute_process(COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/generate_numbers.py ${CMAKE_CURRENT_BINARY_DIR} RESULT_VARIABLE generateResult)
        if(NOT (generateResult EQUAL "0"))
            message(FATAL_ERROR "python and python3 generate_numbers.py failed")
        endif()
    endif()

    set(CMAKE_JNI_TARGET true)
    file(GLOB_RECURSE JAVA_SOURCES src/main/java/*.java ${CMAKE_CURRENT_BINARY_DIR}/generated/*.java)

    if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
        set(CMAKE_JAVA_COMPILE_FLAGS "-h" "${CMAKE_CURRENT_BINARY_DIR}/jniheaders")
        add_jar(wpimath_jar ${JAVA_SOURCES} INCLUDE_JARS ${EJML_JARS} OUTPUT_NAME wpimath)
    else()
        add_jar(wpimath_jar ${JAVA_SOURCES} INCLUDE_JARS ${EJML_JARS} OUTPUT_NAME wpimath GENERATE_NATIVE_HEADERS wpimath_jni_headers)
    endif()

    get_property(WPIMATH_JAR_FILE TARGET wpimath_jar PROPERTY JAR_FILE)
    install(FILES ${WPIMATH_JAR_FILE} DESTINATION "${java_lib_dest}")

    set_property(TARGET wpimath_jar PROPERTY FOLDER "java")
    add_library(wpimathjni ${wpimath_jni_src})
    target_link_libraries(wpimathjni PUBLIC wpimath)

    set_property(TARGET wpimathjni PROPERTY FOLDER "libraries")

    if(${CMAKE_VERSION} VERSION_LESS "3.11.0")
        target_include_directories(wpimathjni PRIVATE ${JNI_INCLUDE_DIRS})
        target_include_directories(wpimathjni PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/jniheaders")
    else()
        target_link_libraries(wpimathjni PRIVATE wpimath_jni_headers)
    endif()
    add_dependencies(wpimathjni wpimath_jar)

    if (MSVC)
        install(TARGETS wpimathjni RUNTIME DESTINATION "${jni_lib_dest}" COMPONENT Runtime)
    endif()

    install(TARGETS wpimathjni EXPORT wpimathjni DESTINATION "${main_lib_dest}")
endif()
