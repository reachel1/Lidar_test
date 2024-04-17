add_library(${PROJECT_NAME}.loosely_lio
        loosely_lio.cc
        cloud_convert.cc
        measure_sync.cc
        )

target_link_libraries(${PROJECT_NAME}.loosely_lio
        ${PROJECT_NAME}.imu_process
        ${PROJECT_NAME}.data_structure
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.inc_ndt
        ${third_party_libs}
        )

add_executable(test_inc_ndt_looselylio
        test_inc_ndt_looselylio.cc
        )

target_link_libraries(test_inc_ndt_looselylio
        ${PROJECT_NAME}.data_structure
        ${PROJECT_NAME}.inc_ndt
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.loosely_lio
        ${PROJECT_NAME}.imu_process
        ${third_party_libs}
        )

