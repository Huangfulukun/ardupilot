// LogStructure.h -- log message definitions for AP_TiltHexa research module
// Part of the libraries/AP_Logger/LogStructure.h per-library pattern
#pragma once

#include <AP_Logger/LogStructure.h>

// THXC - Desired wrench from INDI controller
// @LoggerMessage: THXC
// @Description: TiltHexa Desired Wrench
// @Field: TimeUS: Time since system startup
// @Field: Fxd: Desired Fx
// @Field: Fzd: Desired Fz
// @Field: Mxd: Desired Mx
// @Field: Myd: Desired My
// @Field: Mzd: Desired Mz
struct PACKED log_THXC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Fxd, Fzd, Mxd, Myd, Mzd;
};

// THXA - Model-achieved wrench (w_f = B(x_f) * u_f)
struct PACKED log_THXA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Fxm, Fzm, Mxm, Mym, Mzm;
};

// THXE - Wrench error (w_d - w_f)
struct PACKED log_THXE {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Ex, Ez, ER, EP, EY;
};

// THXT - Tilt angles beta_1..6 (deg)
struct PACKED log_THXT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float B1, B2, B3, B4, B5, B6;
};

// THXF - Thrust forces T_1..6 (N)
struct PACKED log_THXF {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float T1, T2, T3, T4, T5, T6;
};

// THXS - Surface deflections (deg*100)
struct PACKED log_THXS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t AL, AR, RVL, RVR;
};

// THXQ - QP solver diagnostics
struct PACKED log_THXQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t Mode, Stat;
    uint16_t Iter;
    uint32_t Usec;
    uint16_t Sat;
};

// THXI - INDI internal signals
struct PACKED log_THXI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float SigMin, GammaA, GammaT;
};

// THXR - Trajectory reference (Stage 1c extra message)
struct PACKED log_THXR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t phase;
    float pN, pE, pD, vN, vE, vD, yawR;
};

#define LOG_IDS_FROM_TILTHEXA \
    LOG_THXC_MSG, \
    LOG_THXA_MSG, \
    LOG_THXE_MSG, \
    LOG_THXT_MSG, \
    LOG_THXF_MSG, \
    LOG_THXS_MSG, \
    LOG_THXQ_MSG, \
    LOG_THXI_MSG, \
    LOG_THXR_MSG

// clang-format off
#define LOG_STRUCTURE_FROM_TILTHEXA \
    { LOG_THXC_MSG, sizeof(log_THXC), \
      "THXC", "Qfffff", "TimeUS,Fxd,Fzd,Mxd,Myd,Mzd", "s--ttt", "F-0000", true }, \
    { LOG_THXA_MSG, sizeof(log_THXA), \
      "THXA", "Qfffff", "TimeUS,Fxm,Fzm,Mxm,Mym,Mzm", "s--ttt", "F-0000", true }, \
    { LOG_THXE_MSG, sizeof(log_THXE), \
      "THXE", "Qfffff", "TimeUS,Ex,Ez,ER,EP,EY", "s--ttt", "F-0000", true }, \
    { LOG_THXT_MSG, sizeof(log_THXT), \
      "THXT", "Qffffff", "TimeUS,B1,B2,B3,B4,B5,B6", "sdddddd", "F000000", true }, \
    { LOG_THXF_MSG, sizeof(log_THXF), \
      "THXF", "Qffffff", "TimeUS,T1,T2,T3,T4,T5,T6", "s------", "F000000", true }, \
    { LOG_THXS_MSG, sizeof(log_THXS), \
      "THXS", "Qhhhh", "TimeUS,AL,AR,RVL,RVR", "sdddd", "F0000", true }, \
    { LOG_THXQ_MSG, sizeof(log_THXQ), \
      "THXQ", "QBBHIH", "TimeUS,Mode,Stat,Iter,Usec,Sat", "s---s-", "F---F-", true }, \
    { LOG_THXI_MSG, sizeof(log_THXI), \
      "THXI", "Qfff", "TimeUS,SigMin,GammaA,GammaT", "s---", "F---", true }, \
    { LOG_THXR_MSG, sizeof(log_THXR), \
      "THXR", "QBfffffff", "TimeUS,phase,pN,pE,pD,vN,vE,vD,yawR", "s-mmmmmmr", "F-0000000", true },
// clang-format on
