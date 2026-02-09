#ifndef ONEMOTOR_CCM_H
#define ONEMOTOR_CCM_H

#ifndef ONE_MOTOR_LINUX
#include <zephyr/device.h>

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define OM_CCM_ATTR __dtcm_data_section
#else
#define OM_CCM_ATTR
#endif

#else
#define OM_CCM_ATTR
#endif

#endif // ONEMOTOR_CCM_H
