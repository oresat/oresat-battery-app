#ifndef _HIST_H_
#define _HIST_H_

#ifdef __cplusplus
extern "C" {
#endif

void print_batt_hist(void);
void init_batt_hist(void);
void load_latest_batt_hist(pack_t *pack);
bool store_current_batt_hist(void);

#ifdef __cplusplus
}
#endif

#endif /* _HIST_H_ */


