#ifndef _CALIB_H_
#define _CALIB_H_

#ifdef __cplusplus
extern "C" {
#endif

// DEBUG_PRINT is required for ENABLE_NV_MEMORY_UPDATE_CODE to do anything
#if DEBUG_PRINT && CONFIG_ENABLE_NV_MEMORY_UPDATE_CODE
#define NV_WRITE_PROMPT_ENABLED 1
#else
#define NV_WRITE_PROMPT_ENABLED 0
#endif

// DEBUG_PRINT is required for ENABLE_LEARN_COMPLETE to do anything
#if DEBUG_PRINT && CONFIG_ENABLE_LEARN_COMPLETE
#define LEARN_COMPLETE_ENABLED 1
#else
#define LEARN_COMPLETE_ENABLED 0
#endif

#ifdef __cplusplus
}
#endif

#endif /* _CALIB_H_ */
