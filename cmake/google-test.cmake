function(add_test_library_srcs ARG_SRC)
    add_library(${TEST_LIB} ${ARG_SRC})
    target_link_libraries(${TEST_LIB} GTest::gtest ${TEST_EXECUTABLE_COMMON_DEPS})
endfunction()

function(google_test NAME ARG_SRC)
    add_executable(${NAME} ${ARG_SRC})

    # Make sure that gmock always includes the correct gtest/gtest.h.
    target_include_directories("${NAME}" SYSTEM PRIVATE
            "${GMOCK_INCLUDE_DIRS}")
    # todo use GMOCK_LIBRARIES, not gmock_main
    target_link_libraries("${NAME}" PUBLIC GTest::gmock_main)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

function(enable_automatic_test_and_benchmark)
    google_enable_testing()

    file(GLOB_RECURSE ALL_TESTS "src/*_test.cc")
    foreach (ABS_FIL ${ALL_TESTS})
        file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
        # Replace slashes as required for CMP0037.
        string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
        google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB} ${TEST_EXECUTABLE_COMMON_DEPS})
    endforeach ()

    file(GLOB_RECURSE ALL_BENCHMARKS "src/*_benchmark.cc")
    foreach (ABS_FIL ${ALL_BENCHMARKS})
        file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
        get_filename_component(DIR ${REL_FIL} DIRECTORY)
        get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
        # Replace slashes as required for CMP0037.
        string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
        google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
        target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB} glog::glog benchmark::benchmark)
    endforeach ()
endfunction()

macro(google_enable_testing)
    enable_testing()
    #    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    #            ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
    #    find_package(GMock REQUIRED)
endmacro()
