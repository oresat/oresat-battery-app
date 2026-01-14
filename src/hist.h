#ifndef _HIST_H_
#define _HIST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "batt.h"

int hist_init(void);
bool hist_load_current(uint8_t hist_data[HIST_DATA_SIZE]);
bool hist_store_current(const uint8_t hist_data[HIST_DATA_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* _HIST_H_ */


