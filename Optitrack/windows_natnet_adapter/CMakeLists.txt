cmake_minimum_required(VERSION 3.20)
project(windows_natnet_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(OPTITRACK_ADAPTER_WITH_NATNET "Build with the official NatNet SDK" OFF)

add_executable(windows_natnet_adapter
  src/main.cpp
  src/config.cpp
  src/jsonl_writer.cpp
)

target_include_directories(windows_natnet_adapter PRIVATE include)

if(OPTITRACK_ADAPTER_WITH_NATNET)
  if(NOT DEFINED ENV{NATNET_SDK_DIR})
    message(FATAL_ERROR "Set NATNET_SDK_DIR to the OptiTrack NatNet SDK directory")
  endif()

  set(NATNET_SDK_DIR "$ENV{NATNET_SDK_DIR}")

  target_compile_definitions(windows_natnet_adapter PRIVATE OPTITRACK_ADAPTER_WITH_NATNET=1)
  target_include_directories(windows_natnet_adapter PRIVATE "${NATNET_SDK_DIR}/include")
  target_link_directories(windows_natnet_adapter PRIVATE "${NATNET_SDK_DIR}/lib/x64")
  target_link_libraries(windows_natnet_adapter PRIVATE NatNetLib)
endif()
