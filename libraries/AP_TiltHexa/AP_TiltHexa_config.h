// AP_TiltHexa_config.h -- feature flag for AP_TiltHexa research module
#pragma once

#ifndef AP_TILTHEXA_ENABLED
  #include <AP_HAL/AP_HAL_Boards.h>
  #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #define AP_TILTHEXA_ENABLED 1
  #else
    #define AP_TILTHEXA_ENABLED 0
  #endif
#endif