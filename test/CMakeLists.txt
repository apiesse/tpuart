cmake_minimum_required(VERSION 3.16)
project(openknx_tpuart_frame_tests LANGUAGES CXX)

enable_testing()
add_executable(frame_test frame_test.cpp)
target_compile_features(frame_test PRIVATE cxx_std_14)
target_include_directories(frame_test PRIVATE ../src)
add_test(NAME frame_test COMMAND frame_test)

file(GLOB driver_sources ../src/TPUart/*.cpp)
add_executable(transmit_result_test transmit_result_test.cpp ${driver_sources})
target_compile_features(transmit_result_test PRIVATE cxx_std_14)
target_include_directories(transmit_result_test PRIVATE stubs ../src)
add_test(NAME transmit_result_test COMMAND transmit_result_test)

add_executable(espidf_receive_test espidf_receive_test.cpp ${driver_sources}
    ../src/TPUart/Interface/EspIdf.cpp)
target_compile_features(espidf_receive_test PRIVATE cxx_std_14)
target_compile_definitions(espidf_receive_test PRIVATE ESP_PLATFORM)
# Match Xtensa's default char signedness when exercising the ESP-IDF backend.
target_compile_options(espidf_receive_test PRIVATE -funsigned-char)
target_include_directories(espidf_receive_test PRIVATE espidf_stubs ../src)
add_test(NAME espidf_receive_test COMMAND espidf_receive_test)

option(ENABLE_SANITIZERS "Run host tests with ASan and UBSan" OFF)
if(ENABLE_SANITIZERS)
    foreach(target frame_test transmit_result_test espidf_receive_test)
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endforeach()
endif()
