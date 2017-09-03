#ifndef _TASKS_H_
#define _TASKS_H_

#include "pt.h"

// Long running tasks
PT_THREAD( lepton_task(struct pt *pt));
PT_THREAD( uart_task(struct pt *pt));
PT_THREAD( usb_task(struct pt *pt));
PT_THREAD( button_task(struct pt *pt));

// Temporary tasks
PT_THREAD( uart_lepton_send(struct pt *pt,char * buffer));

// Synchronization helpers
lepton_buffer* dequeue_lepton_buffer(void);
uint32_t get_lepton_buffer(lepton_buffer **buffer);

//Other
void change_overlay_mode(void);
void rgb2yuv(const rgb_t val, uint8_t *y, uint8_t *u, uint8_t *v);

#endif
