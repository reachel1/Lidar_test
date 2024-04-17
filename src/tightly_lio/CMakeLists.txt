add_library(${PROJECT_NAME}.tightly_lio
        lio_iekf.cc
        )

target_link_libraries(${PROJECT_NAME}.tightly_lio
        ${PROJECT_NAME}.inc_ndt
        ${PROJECT_NAME}.imu_process
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.loosely_lio
        ${PROJECT_NAME}.data_structure
        ${third_party_libs}
        )

add_executable(test_inc_ndt_tightlylio
        test_inc_ndt_tightlylio.cc
        )

target_link_libraries(test_inc_ndt_tightlylio
        ${PROJECT_NAME}.data_structure
        ${PROJECT_NAME}.inc_ndt
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.loosely_lio
        ${PROJECT_NAME}.imu_process
        ${PROJECT_NAME}.tightly_lio
        ${third_party_libs}
        )
