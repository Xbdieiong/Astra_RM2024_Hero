#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP   1650        //右键，高速

#define FRIC_DOWN  1440     //左键，低速

#define FRIC_MIDDLE  1480

#define FRIC_1V1_UP    1650   //1V1的弹速，右键，高速
#define FRIC_1V1_DOWN  1650   //1V1的弹速，左键，低速

#define FRIC_OFF 1000

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
