set(bh_library_links
    sat-bh-metrics.cc
    sat-bh-metrics.h
    sat-bh-obc.cc
    sat-bh-obc.h
    sat-bh-helper.cc
    sat-bh-helper.h
)

foreach(bh_library_file IN LISTS bh_library_links)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${bh_library_file}"
       AND NOT IS_SYMLINK "${CMAKE_CURRENT_SOURCE_DIR}/${bh_library_file}")
        message(FATAL_ERROR
            "${CMAKE_CURRENT_SOURCE_DIR}/${bh_library_file} must remain a symlink to "
            "contrib/satellite/helper/${bh_library_file}. Edit or copy the file in "
            "contrib/satellite/helper so the satellite library uses the new code.")
    endif()
endforeach()

build_exec(
    EXECNAME sat-bh-example
    EXECNAME_PREFIX scratch_bh_dynamic_Codes_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/sat-bh-example.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/bh_dynamic/Codes/
)

build_exec(
    EXECNAME sat-bh-2d-footprint
    EXECNAME_PREFIX scratch_bh_dynamic_Codes_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/sat-bh-2d-footprint.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/bh_dynamic/Codes/
)
