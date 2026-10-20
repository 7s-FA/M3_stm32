#ifndef IMU_DIAGNOSTICS_H
#define IMU_DIAGNOSTICS_H
#include <rcl/rcl.h>
#include "imu.h"
rcl_ret_t IMUDiagnostics_Init(rcl_node_t *node);
void IMUDiagnostics_Update(rcl_ret_t spin_result, const IMU_Status_t *status);
void IMUDiagnostics_Fini(rcl_node_t *node);
#endif
