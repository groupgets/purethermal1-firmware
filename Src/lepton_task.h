#ifndef _LEPTON_TASK_H_
#define _LEPTON_TASK_H_

#include "pt.h"

uint32_t get_lepton_buffer(lepton_buffer **buffer);

PT_THREAD( lepton_task(struct pt *pt));

#endif


