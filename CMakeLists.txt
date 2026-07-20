cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD "oresat_stm32_battery_card")

set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(PATCHLEVEL "0")
set(VERSION_TWEAK "0")
set(EXTRAVERSION "00000000")

find_package(Git QUIET REQUIRED)

execute_process(
    COMMAND "${GIT_EXECUTABLE}" describe --always HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE EXTRAVERSION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)

set_property(GLOBAL APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/.git/index")

#
# utilize settings to autogenerate a header file to adjust the build inside the C code
#
configure_file (
   "${CMAKE_SOURCE_DIR}/src/VERSION.in"
   "${CMAKE_SOURCE_DIR}/VERSION"
   )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app_battery LANGUAGES C)

#execute_process(COMMAND bash -c "oresat-configs canopennode ${CMAKE_CURRENT_SOURCE_DIR}/od.yaml ${CMAKE_CURRENT_SOURCE_DIR}/../../common/od.yaml -d gen"

include_directories("${PROJECT_SOURCE_DIR}") # needed so c code can find this header

target_sources(app PRIVATE
  src/main.c
  src/batt.c
  src/can.c
  src/calib.c
  src/hist.c
  src/max17205_intf.c
  objdict/CO_OD.c
  drivers/sensor/maxim/max17205/max17205.c
  drivers/sensor/maxim/max17205/max17205_emul.c
)

zephyr_include_directories(objdict)
