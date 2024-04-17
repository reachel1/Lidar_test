add_library(${PROJECT_NAME}.gins
        # ieskf/nav_state_manifold.cc
        # ieskf/ieskf.cc
        utm_convert.cc
        )

target_link_libraries(${PROJECT_NAME}.gins
        glog gflags 
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.imu_process  
        ${PROJECT_NAME}.tools
        )


add_executable(run_eskf_gins run_eskf_gins.cc)
target_link_libraries(run_eskf_gins
        glog gflags 
        ${PROJECT_NAME}.common 
        ${PROJECT_NAME}.gins
        ${PROJECT_NAME}.imu_process  
        ${PROJECT_NAME}.tools
        )

add_executable(process_gnss process_gnss.cc)
target_link_libraries(process_gnss
        glog gflags  
        ${PROJECT_NAME}.gins
        ${PROJECT_NAME}.imu_process  
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.common
        )



