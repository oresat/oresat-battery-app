# oresat-app-battery

This repository contains the Oresat Zephyr app for the battery card for Oresat v1.0.

## Summary
This application monitors and reports over CAN the telemetry related to the two dual cell battery packs on the battery card.

Data include:

- the SOC (state of charge, a percentage)
- current and full capacity
- temperature
- current draw
- voltage per cell
- voltage per pack
- time to full and time to empty
- status bit flags

## Compile-time options
Kconfig symbol values are the primary way Zephyr uses for compile-time customization.
While it is also possible to use legacy C preprocessor symbols, the Kconfig system has the advantage of a centralized mechanism
with powerful tools for coordinating between symbols defined at any level of Zephyr -- from the kernel to the application code itself,
and all layers in-between.

> **Note: Kconfig files always omit the `CONFIG_` prefix, but configuration files (`prj.conf`, `overlay_debug.conf`, etc.) and source code requires them.**

The default symbol values enable the application to be built for use in the lab with serial logging enabled at the INF level, and no battery heaters.
TODO: Prior to launch, this can be changed (with a new PR) to make it the launch-compatible settings, or we could define a Kconfig
fragment specific for launch.

### Kconfig symbols
- `CONFIG_APP_BATTERY_LOG_LEVEL_INF`: The final suffix defines the default level for application logging. Set the value to `n` to disable logging.
- `CONFIG_ACCESS_FLASH_STM32`: A non-user-configurable setting used to enable STM32 HAL subsystems we need if building for an STM32 board.
- `CONFIG_VERBOSE_DEBUG`: Enables dumping of capacity history, its location in flash, and all the non-volatile MAX17205 registers.
- `CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE`: Enables storing of a subset of volatile MAX17205 registers to MAX17205 non-volatile registers; limited to 7 writes over IC lifetime.
- `CONFIG_ENABLE_LEARN_COMPLETE`: Enables detection of reaching the final calibration stage.
- `CONFIG_ENABLE_HEATERS`: Enables battery heater state machine. Not recommended with battery boards older than v3. Defaults to disabled.
- `CONFIG_ENABLE_CHARGING_CONTROL`: Enables legacy code for setting and clearing the pack charge and discharge lines. TODO: remove this?
- `CONFIG_HIST_STORE_PROMPT`: Enables prompt to erase all history, test the history system, store, or abandon a capacity history update.

Battery history includes both the MAX17205 life logging data (up to 203 snapshots of NV registers 1A0h to 1AFh; see datasheet),
as well as our custom periodic logging of MAX17205_AD_MIXCAP and MAX17205_AD_REPCAP used for reset recovery.

`CONFIG_NV_MEMORY_UPDATE_CODE` and `CONFIG_ENABLE_LEARN_COMPLETE` are used for a special build to be run in the lab.
They are enabled using the `overlay_calib.conf` Kconfig fragment for convenience.
Use these on a new battery pack to get the MAX17205 fuel gauge chips through multiple full charge / discharge learning cycles.
This is needed for them to learn the battery characteristics for proper operation of the ModelGauge m5 algorithm's coulomb counting mode.
Without this step, the default, much less accurate voltage-based mode is used by the MAX17205. The latter is what was used on Oresat 0 and Oresat 0.5,
resulting in very untrustworthy battery telemetry.

> **Note: the MAX17205 has a very limited number of NV (non-volatile) memory write cycles: 7 total.**

### Kconfig fragments
Kconfig fragments are used to define a set of Kconfig value changes that override the default settings in `prj.conf` and all the `Kconfig` files in the build.
We define two for user convenience:
- `overlay_calib.conf`: Override Kconfig fragment for building the application with calibration enabled (used prior to launch).
- `overlay_debug.conf`: Override Kconfig fragment for building the application with lots of debug logging and ability to use gdb well.


# Building and flashing
Ensure you are in the `battery` directory (`cd src/oresat/firmware/apps/battery`) prior to building.

> NOTE:
>   The `oresat_stm32_battery_card` is the default. It normally does not need to be specified as shown below,
>   except for the important exception of enabling MCUboot. In that case, it is mandatory for Zephyr 4.2.0.

> NOTE:
>   The stm32 version of the battery card does not have MCUboot support.

| Board         | Build Example                                         |
| ------------- | ----------------------------------------------------- |
| oresat_stm32_battery_card | `west build -p always -b oresat_stm32_battery_card/mcxn947/cpu0` |
| oresat_stm32_battery_card with calibration | `west build -p always -b oresat_stm32_battery_card/mcxn947/cpu0 -- -DEXTRA_CONF_FILE=overlay_calib.conf` |
| oresat_stm32_battery_card debug and calibration | `west build -p always -b oresat_stm32_battery_card/mcxn947/cpu0 -- -DEXTRA_CONF_FILE=1overlay_calib.conf;overlay_debug.conf` |

> NOTE: the section below only gives general instructions. Specific steps below (like for setting the CAN node id) are self-contained
>   in the section.

## First build

For the first build:
```
$ west build -p
$ west flash --erase -r openocd
```
This does a clean build and makes sure the full flash is empty.

## Building
A simple default build using `west` that forces a complete rebuild:
```
$ west build -p
```
> **Note: omit `-p` for an incremental build.**

A build that changes a single Kconfig symbol value:
```
$west build -- -DCONFIG_APP_BATTERY_LOG_LEVEL_DBG=y
```

In order to use a Kconfig fragment, specify it using the `EXTRA_CONF_FILE` CMake variable.
With `west`, do the following for a single fragment:
```
$ west build -- -DEXTRA_CONF_FILE='overlay_calib.conf'
```

Or for more than one Kconfig fragment, do this:
```
$ west build -- -DEXTRA_CONF_FILE='overlay_calib.conf;overlay_debug.conf'
```

## Setting the CAN node id
For the stm32 version of the battery card, follow the process as documented for the
[ChibiOS version](https://github.com/oresat/oresat-firmware/blob/master/toolchain/flash_node_id.py).

# Shortcomings of original Oresat battery app
- original code was not using all register values from the Maxim Wizard software with no explanation as to why
- no learning cycles performed -- full charge/discharge never done to tune the fuel guage prior to launch
- lots of duplicate code differing only in pack1 vs. pack2
- magic numbers in code
- no use of firmware reset to allow for runtime experimentation with register values

## Shortcomings of v2.1 and v3.0 battery boards
- cells 1 and 2 in each pack tend to get out of balance quickly (at least in the lab)
- unexpected and uninvestigated charge behavior observed on the EU (v3.0 pack) between packs 1 and 2
- Vov (overvoltage) cutoff is slightly too low at 4.1v but cells are only full at 4.2v

## Limitations of the MAX17205
- the datasheet is large, complex, incomplete, and not well organized -- important info also in various app notes
- ModelGauge m5 EZ model is not well documented -- there is no clear list of how to get the chip in that mode
- works best if one pays Maxim to generate custom characteristic data for the specific cells being used (cost-prohibitive)
- built-in ModelGauge m5 EZ battery model does not fit Oresat's cells well
- acceptable performance of EZ mode requires the chip to reach learn level 7, which requires 7 full in-lab charge/discharge cycles with NO resets the entire training period, followed by writing to NV registers
- only 7 writes possible to the nonvolatile registers; this mandates adding code to do runtime volatile configuration when performing experiments
- Coulomb-based fuel gauge tracking does not tolerate our daily reset behavior in orbit (meant to recover from hard-faults in orbit)
- battery balancing circuit is limited to operating only when close to the end of a charge cycle

## Recovery from limitations
- fully train and store to EV with:
  - `MAX17205_AD_NLEARNCFG` = 7 (learning complete)
  - `MAX17205_AD_NFULLCAPNOM` = 0x1A22 (higher than Wizard value of 0x1794; found during full learning on 2 battery boards; redo for Oresat 1.0)
  - `MAX17205_AD_MIXCAP` and `MAX17205_AD_REPCAP` = 2600 mAh (rated full capacity of the cells we use)
- add logging of `MAX17205_AD_MIXCAP`, `MAX17205_AD_REPCAP`, number of reset cycles, and current cycle run time to a page in flash on the STM processor
- on reset use last good history entry to restore coulomb-counting
- balancing may or may not be a long-term problem in orbit; need latest Oresat 0.5 telemetry for analysis

### Battery characteristic learning procedure
- battery card must be setup in a standalone mode:
  - connect external 3.3v supply to `OPD_PWR`
  - connect a USB to I2C dongle to the Oresat Power Domain Controller MAX7310's pins using `OPD_SDA`, `OPD_SCL`, and `GND` to enable Vbusp
  - write the MAX7310's direction register and the output register:

    1. register 0x03 --> 0x07 (bits 0, 1, 2 = input, remaining bits = output)
    2. register 0x01 --> 0 for reset / power off
    3. register 0x01 --> 0x98 for ON/OFF = 1, CB-RESET = 1, UART_EN = 1

  - no resetting of the processor is allowed or learning progress is entirely lost
  - no totally full discharge is allowed below Vuv = 2.5v (under voltage) or processor will reset
  - set an external supply to Vbus for charging to 4.2v with current limited to 1.25A per pack (2.5A total)
  - connect a dummy load to Vbus, such as a 20 ohm 10 watt power resistor, to drain the pack in a few hours rather than days
  - cycle pack through:

    1. from fully charged (indicated when time to full jumps from 0 to 368634 and SOC = 100%)
    2. to nearly fully discharged (Vbatt in either pack <= 2.85v; `Batteries are critically low` in terminal)
    3. observe `MAX17205_AD_LEARNCFG` debug output and confirm it increments from 0 to 7 over the 7 charge/discharge cycles
    4. when learning complete, debug output = `Learning state set = 7`
    5. `Write NV memory on MAX17205 for pack (1 or 2) ? y/n?` will appear; you have 15 seconds to say `y` otherwise it will not be written

- cells should be well-balanced before starting; do so by manually charging each cell using their test points while OPD is shut down / safety switches closed
- TODO: automation of USB-interface power supply and scripting of learning cycles would be very helpful, but must be monitored for fire safety reasons

### Daily satellite reset recovery
- on reset:
  - read from logging page
  - restore newest entry that also passes CRC checks
  - this ensures the fuel gauge restarts tracking where it left off
- a limitation is that a long off time due to prelaunch processing or simply leaving the battery board on the shelf will result in loss of accuracy
- the voltage-based fuel gauge should eventually correct the discrepancy (TODO: verify this)

## Volatile vs. Non-volatile registers
The MAX17205 has two sets of registers: RAM-based vs. Non-volatile (NV).
The RAM-based registers, sometimes called NV-RAM registers or shadow registers, can change values over time while running.
The Non-volatile registers are loaded into the RAM-based registers at hardware reset and used as a starting point.

Because the number of write cycles allowed to the NV registers is limited to 7, writing should only be done when certain that the values being written are correct.
This makes NV writes useless for experimentation.

Luckily, the MAX17205 has both a hardware and a firmware reset.

- hardware reset loads RAM registers from NV registers
- firmware reset simply restarts the ModelGauge m5 algorithm with the current RAM registers left alone

We leverage the firmware reset functionality in the `nv_ram_write()` function to make temporary changes to the registers without wasting NV write cycles.
This function first validates the current contents. If they match, then nothing further is done. Otherwise the registers are written and then validated again.
This will be useful in orbit to change behavior if either the NV writes are already exhausted, or we do not want to risk using them.

# Application Main Directory
## Directory Structure
This folder contains the root of an applications sources. It contains
several files and folders:

- `/src`: The directory where application code is located.
  - `main.c`: The main program file called by Zephyr.
  - `batt.c`: Main thread for managing the battery packs and updating the CAN structures for the CAN thread to transmit to C3.
  - `calib.c`: Auxilary thread for monitoring status of battery learning procedure, when enabled.
  - `can.c`: Auxilary thread for processing CAN transactions.
  - `hist.c`: Code for periodically storing in flash a running history of battery capacity data, and for loading the most recent upon boot.
  - `max17205_intf.c`: Interface between application code and the max17205 Zephyr driver.
  - `*.h`: Header files for each of the above c files
  - `diag.h`: Header used during build to output pragma messages reflecting the configuration. Disabled currently.
- `CMakeLists.txt`: The main build file for the application.
- `Kconfig`: Configuration variable definitions that can be used from source code to control compile-time and run-time behavior.
- `prj.conf`: Default values for configuration variables, both local ones defined in `Kconfig` as well as others in other folders and in Zephyr itself.
- `overlay_calib.conf`: Override Kconfig fragment for building the application with calibration enabled (used prior to launch).
- `overlay_debug.conf`: Override Kconfig fragment for building the application with lots of debug logging and ability to use gdb well.
- `sample.yaml`: Test infrastructure configuration. Currently not used.
- `/boards`: Folder to locally define a per-application set of board files.
  - `/oresat_stm32_battery_card`: Folder for the v3 and earlier STM32-based battery cards.
    - `board.cmake`: Configuration for SWD hardware (OpenOCD and JLink).
    - `board.yml`: Required high level description of the board.
    - `Kconfig.defconfig`: Kconfig file that ensures SPI interrupts are enabled on the STM32.
    - `Kconfig.oresat_stm32_battery_card`:  Kconfig file that enables the correct SoC for this board.
    - `oresat_stm32_battery_card_defconfig`: Baseline configuration variable settings for this board.
    - `oresat_stm32_battery_card.yaml`: Test infrastructure configuration. Currently not used.
    - `oresat_stm32_battery_card.dts`: Board-specific device tree additions and changes from the SoC baseline.
- `/drivers`: Folder to locally define a per-application set of device drivers.
  - `/sensor`: Folder for sensor drivers.
    - `/maxim`: Folder for Maxim sensors.
      - `/max17205`: Folder for the Maxim sensor part number.
        - `CMakeLists.txt`: The build file for the driver.
        - `Kconfig`: Kconfig for the driver. This enables the CONFIG_MAX17205 symbol if the device tree enables the sensor. If so, it also ensures CONFIG_I2C is defined.
        - `max17205.c`: Source code for the driver.
        - `max17205.h`: Header file for the driver.
- `/dts`: Folder containing device tree bindings.
  - `/bindings`: Folder for the bindings themselves.
    - `battery_packs.yaml`: Custom device tree symbol definitions for board-specific GPIO lines.
    - `maxim,max17205.yaml`: Custom device tree symbol definitions for device driver configuration.

