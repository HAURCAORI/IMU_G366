cmake_minimum_required(VERSION 3.8)

project("IMU_G366")

add_executable(IMU_G366
    main.cpp
    main_ctx.cpp
    uart.cpp
    gyro_api.cpp
)

target_include_directories(IMU_G366 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
target_include_directories(IMU_G366 PRIVATE ${ZMQ_INCLUDE_DIRS})

find_package(Threads REQUIRED)
target_link_libraries(IMU_G366 PRIVATE Threads::Threads ${ZMQ_LIBRARIES})



if(CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET IMU_G366 PROPERTY CXX_STANDARD 20)
endif()
