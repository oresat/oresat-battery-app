#pragma once
#ifndef _DIAG_H_
#define _DIAG_H_

// display configuration during the build process
// this is noisy, so turn off by default
#if 0
#if DEBUG_PRINT
#pragma message("Debug messages enabled")
#else
#pragma message("Debug messages disabled")
#endif

#if VERBOSE_DEBUG
#pragma message("Verbose debug messages enabled")
#else
#pragma message("Verbose debug messages disabled")
#endif

// If batt_nv_programing_cfg registers do not match current, rewrite the RAM shadow then prompt to write to NV.
#if CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE
#pragma message("NV memory update code enabled")
#else
#pragma message("NV memory update code disabled")
#endif

// If state of charge is known to be full, set LS bits D6-D0 of LearnCfg register to 0b111
// and write MixCap and RepCap registers to 2600.
#if CONFIG_ENABLE_LEARN_COMPLETE
#pragma message("Enable learn complete enabled")
#else
#pragma message("Enable learn complete disabled")
#endif

// Recommend setting ENABLE_HEADERS to 0 for battery board v2.1. Otherwise, brownouts can occur, causing
// the C3 to reboot the battery board.
#if CONFIG_ENABLE_HEATERS
#pragma message("Heaters enabled")
#else
#pragma message("Heaters disabled")
#endif
#endif

#endif // _DIAG_H_
