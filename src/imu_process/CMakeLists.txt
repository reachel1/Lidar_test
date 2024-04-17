add_library(${PROJECT_NAME}.imu_process
        static_imu_init.cc
        )

target_link_libraries(${PROJECT_NAME}.imu_process
        glog gflags ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )

