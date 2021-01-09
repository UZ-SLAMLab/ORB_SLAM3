cmake_minimum_required(VERSION 3.18)

project(ORB_SLAM3_Tools)

find_package(spdlog CONFIG REQUIRED)

add_executable(text_to_binary_vocab text_to_binary_vocab.cpp)

target_link_libraries(text_to_binary_vocab 
                      PRIVATE 
                      spdlog::spdlog
                      ORB_SLAM::ORB_SLAM3)

