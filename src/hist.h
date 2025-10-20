#ifndef _HIST_H_
#define _HIST_H_

#ifdef __cplusplus
extern "C" {
#endif

void batt_hist_init(void);
void batt_hist_load_latest(pack_t *pack);
bool batt_hist_store_current(void);

#ifdef __cplusplus
}
#endif

#endif /* _HIST_H_ */


