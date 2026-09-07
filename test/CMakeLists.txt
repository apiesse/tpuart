cmake_minimum_required(VERSION 3.16)
project(openknx_tpuart_frame_tests LANGUAGES CXX)

enable_testing()
add_executable(frame_test frame_test.cpp)
target_compile_features(frame_test PRIVATE cxx_std_14)
target_include_directories(frame_test PRIVATE ../src)
add_test(NAME frame_test COMMAND frame_test)
