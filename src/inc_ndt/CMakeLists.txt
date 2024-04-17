add_library(${PROJECT_NAME}.inc_ndt     
        incremental_ndt_lo.cc
        ndt_inc.cc
        )

target_link_libraries(${PROJECT_NAME}.inc_ndt
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.data_structure
        ${third_party_libs}
        )



add_executable(test_inc_ndt_lo
        test_inc_ndt_lo.cc
        )

target_link_libraries(test_inc_ndt_lo
        ${PROJECT_NAME}.data_structure
        ${PROJECT_NAME}.inc_ndt
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

