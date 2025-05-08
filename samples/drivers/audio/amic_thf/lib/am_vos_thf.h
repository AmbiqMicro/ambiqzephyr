//*****************************************************************************
//
//! @file am_vos_thf.h
//!
//! @brief Sensory THF engine defines
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_VOS_SENSORYTHF_H
#define AM_VOS_SENSORYTHF_H

#include <stdint.h>

#define THF_COMMAND_TIMEOUT_MS      (5000)
#define THF_DETECT_WAKEWORD         1
#define THF_DETECT_COMMAND          2

//*****************************************************************************
// Tuning Driver function declaration
//*****************************************************************************
errors_t SensoryInitialize(void); 
errors_t SensoryTerminate(void);
#endif
