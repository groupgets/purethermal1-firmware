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
PT_THREAD( rgb_to_yuv(struct pt *pt, lepton_buffer *restrict rgb, yuv422_buffer_t *restrict buffer));

// Synchronization helpers
uint32_t get_lepton_buffer(lepton_buffer **buffer);
uint32_t get_lepton_buffer_yuv(yuv422_buffer_t **buffer);

//Other
void change_overlay_mode(void);

#endif
