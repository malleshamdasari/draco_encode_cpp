cmake_minimum_required(VERSION 3.16)

project(DracoEncode)
find_package(draco REQUIRED)

add_executable(draco_encode ./encode.cpp)
add_executable(server ./server.cpp)
add_executable(client ./client.cpp)
target_link_libraries(draco_encode -Wl,--whole-archive draco -Wl,--no-whole-archive)
target_link_libraries(server -Wl,--whole-archive draco -Wl,--no-whole-archive)
target_link_libraries(client -Wl,--whole-archive draco -Wl,--no-whole-archive)
