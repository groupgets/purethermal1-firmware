#ifndef _LEPTON_TASK_H_
#define _LEPTON_TASK_H_

#include "pt.h"

lepton_buffer * get_lepton_buffer(void);
int get_lepton_frame(void);

PT_THREAD( lepton_task(struct pt *pt));

#endif


