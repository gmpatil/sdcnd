project(PID C CXX)

cmake_minimum_required (VERSION 3.5)
# GMP Added below line
FIND_PACKAGE ( Threads REQUIRED )
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pid ${sources})
# GMP Added below CMAKE_THREAD_LIBS_INIT parameter
target_link_libraries(pid z ssl uv uWS ${CMAKE_THREAD_LIBS_INIT})
