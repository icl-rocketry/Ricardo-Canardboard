/*
**********************
* PINS               *
**********************
 */
#pragma once

namespace PinMap{

    static constexpr int ServoPWM1 = 38;
    static constexpr int ServoPWM2 = 9;
    static constexpr int ServoPWM3 = 40;

    static constexpr int MISO = 33;
    static constexpr int MOSI = 34;
    static constexpr int SCLK = 35;
    static constexpr int MagCs = 48;
    static constexpr int ImuCs = 47;

    static constexpr int SDMISO = 15;
    static constexpr int SDMOSI = 17;
    static constexpr int SDSCLK = 16;
    static constexpr int SDCs = 18;
    static constexpr int SDDet = 14;

    static constexpr int BuckEN = 39;
    static constexpr int BuckPGOOD = 42;

    static constexpr int TxCan = 36;
    static constexpr int RxCan = 37;

    static constexpr int BuckOutputV = -1;
};


