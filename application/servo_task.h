#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

extern int servor_key_count;
extern int servor_open_flag;
extern int last_servor_open_flag;
extern int last_s_servor;
extern int servor_real_open_ui;
extern void servo_task(void const * argument);

#endif
