cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD "oresat_mcxn947_battery_card/mcxn947/cpu0")
#set(BOARD "oresat_stm32_battery_card")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app_battery LANGUAGES C)

#execute_process(COMMAND bash -c "oresat-configs canopennode ${CMAKE_CURRENT_SOURCE_DIR}/od.yaml ${CMAKE_CURRENT_SOURCE_DIR}/../../common/od.yaml -d gen"

target_sources(app PRIVATE
  src/main.c
  src/batt.c
  src/can.c
  src/calib.c
  src/hist.c
  src/max17205_intf.c
  objdict/CO_OD.c
  drivers/sensor/maxim/max17205/max17205.c
)

zephyr_include_directories(objdict)
