cmake_minimum_required(VERSION 3.14.7)

project(pcl_programs VERSION 0.1.0 LANGUAGES CXX C)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED COMPONENTS CXX)

include(FetchContent)

FetchContent_Declare(bvh URL "https://github.com/madmann91/bvh/archive/refs/heads/master.zip")
FetchContent_MakeAvailable(bvh)

add_library(pcl_programs_common
  common/arg_parser.h
  common/arg_parser.cpp
  common/bvh_model.h
  common/bvh_model.cpp
  common/stb_image_write.h
  common/stb_image_write.c)

target_link_libraries(pcl_programs_common PUBLIC bvh)

target_compile_features(pcl_programs_common PUBLIC cxx_std_17)

function(add_pcl_program name)

  set(target pcl_${name})

  add_executable(${target} ${ARGN})

  target_link_libraries(${target} PRIVATE ${PCL_LIBRARIES} pcl_programs_common)

  target_compile_features(${target} PRIVATE cxx_std_17)

  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -Wfatal-errors)
  endif(CMAKE_COMPILER_IS_GNUCXX)

endfunction(add_pcl_program name)

add_pcl_program(spawn_box       pcl_spawn_box.cpp)
add_pcl_program(spawn_random1f  pcl_spawn_random1f.cpp)
add_pcl_program(spawn_uniform1f pcl_spawn_uniform1f.cpp)
add_pcl_program(spawn_uniform3f pcl_spawn_uniform3f.cpp)
add_pcl_program(render          pcl_render.cpp)

add_pcl_program(shell
  shell/main.cpp
  shell/lexer.h
  shell/lexer.cpp)
