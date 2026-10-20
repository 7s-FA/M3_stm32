#ifndef IMU_DIAGNOSTICS_H
#define IMU_DIAGNOSTICS_H
#include <rcl/rcl.h>
rcl_ret_t IMUDiagnostics_Init(rcl_node_t *node);
void IMUDiagnostics_Update(rcl_ret_t spin_result);
void IMUDiagnostics_Fini(rcl_node_t *node);
#endif
