cmake_minimum_required(VERSION 3.20)

project(DracoEncode)
find_package(draco REQUIRED)

project(TestVisualizer LANGUAGES C CXX)

# Find installed Open3D, which exports Open3D::Open3D
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/cmake)
endif()
if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif()

add_executable(draco_encode ./encode.cpp)
add_executable(server ./server.cpp)
add_executable(client ./client.cpp)
add_executable(TestVisualizer ./TestVisualizer.cpp)
#add_executable(rply ./rplyCustom.cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hdr)

target_link_libraries(draco_encode -Wl,--whole-archive draco -Wl,--no-whole-archive)
target_link_libraries(server -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)
target_link_libraries(client -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D  )

target_link_libraries(TestVisualizer Open3D::Open3D)

