cmake_minimum_required(VERSION 3.20)

project(test)
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

include_directories(/home/allan/draco_encode_cpp)

file(GLOB SOURCES
    /home/allan/draco_encode_cpp/plyEncoderCustom.h
    /home/allan/draco_encode_cpp/plyEncoderCustom.cpp
)

#add_library(rply STATIC rplyCustom.cpp rplyCustom.h)

#add_executable(draco_encode ./encode.cpp)
#add_executable(server ./server.cpp ./plyEncoderCustom.cpp ./rplyCustom.cpp ./stdio_file_writerCustom.cpp)
#add_executable(client ./client.cpp ./plyEncoderCustom.cpp ./rplyCustom.cpp ./stdio_file_writerCustom.cpp)
#add_executable(TestVisualizer ./TestVisualizer.cpp)
#add_executable(test ./test.cpp ./plyEncoderCustom.cpp ./rplyCustom.cpp ./stdio_file_writerCustom.cpp)
#add_executable(test ./test.cpp ./plyEncoderCustom.cpp ./rplyCustom.cpp)
add_executable(test ./test.cpp ${SOURCES})
#add_executable(test ./test.cpp )

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hdr)

target_link_libraries(test -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)


#target_link_libraries(draco_encode -Wl,--whole-archive draco -Wl,--no-whole-archive )
#target_link_libraries(server -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)
#target_link_libraries(client -Wl,--whole-archive draco -Wl,--no-whole-archive Open3D::Open3D)
#target_link_libraries(TestVisualizer Open3D::Open3D)



add_definitions(-O9)

#target_compile_options(test PUBLIC
  #"-Werror" # Do not comment this out after Project 8!
  #"-ggdb"
  #"-O0"
  #"-Wunreachable-code"
  #"-g"
  #"-Wextra"
  #"-Wall"
  #"-Wfloat-equal"
  #"-Wconversion"
  #"-Wparentheses"
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
  #)