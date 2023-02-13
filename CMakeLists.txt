set(CMAKE_INSTALL_PREFIX /home/sc/open3d_install_0.16.0)
cmake_minimum_required(VERSION 3.20)

project(test)
find_package(draco REQUIRED)
find_package(Open3D 0.16.0 REQUIRED)

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

include_directories(../draco_encode_cpp)


add_executable(server ./server.cpp ${SOURCES})
add_executable(client ./client.cpp ${SOURCES})
add_executable(test ./test.cpp ${SOURCES})

target_link_libraries(test -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)
target_link_libraries(server -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)
target_link_libraries(client -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)

#add_definitions(-O3)

target_compile_options(test PUBLIC
  #"-Werror" # Do not comment this out after Project 8!
  "-ggdb"
  "-O3"
  "-Wunreachable-code"
  "-g"
  "-Wextra"
  "-Wall"
  "-Wfloat-equal"
  "-Wconversion"
  "-Wparentheses"
  #"-pedantic"
  #"-Wunused-parameter"
  #"-Wunused-variable"
  #"-Wreturn-type"
  #"-Wunused-function"
  #"-Wredundant-decls"
  #"-Wreturn-type"
  #"-Wredundant-decls"
  #"-Wunused-value"
  #"-Wswitch-default"
  #"-Wuninitialized" "-Winit-self"
  )