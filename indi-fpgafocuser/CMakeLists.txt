cmake_minimum_required(VERSION 3.0.0)
PROJECT(indi-fpga-focuser CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

set (CMAKE_CXX_STANDARD 14)
include(GNUInstallDirs)

set (VERSION_MAJOR 2)
set (VERSION_MINOR 6)

find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_fpga_focuser.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_fpga_focuser.xml)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

set(FPGA_LIBRARIES "libfpgafocuser.a")

################ Astroberry Focuser ################
set(indi_astroberry_focuser_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/fpga_focuser.cpp
   )

add_executable(indi_fpga_focuser ${indi_astroberry_focuser_SRCS})
target_link_libraries(indi_fpga_focuser ${INDI_DRIVER_LIBRARIES} ${FPGA_LIBRARIES})
install(TARGETS indi_fpga_focuser RUNTIME DESTINATION bin )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_fpga_focuser.xml DESTINATION ${INDI_DATA_DIR})

