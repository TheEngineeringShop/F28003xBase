#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C"
{
#endif


#include <CLAmath.h>
#include <math.h>

#include "cla_shared.h"

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "version.h"

#include "canbus.h"
#include "mcanbus.h"

#include "cana_msgs.h"  // rides on CANA
#include "mcan_msgs.h" // rides on MCAN


// Globals - Timer element
volatile uint16_t cpuTimer0IntCount;


#endif
