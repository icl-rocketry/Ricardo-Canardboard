/*
**********************
* PINS               *
**********************
 */
#pragma once

namespace PinMap{

    static constexpr int ServoPWM1 = 43;
    static constexpr int ServoPWM2 = 14;
    static constexpr int ServoPWM3 = 45;

    static constexpr int MISO = 38;
    static constexpr int MOSI = 39;
    static constexpr int SCLK = 40;
    static constexpr int MagCs = 36;
    static constexpr int ImuCs = 37;

    static constexpr int SDMISO = 21;
    static constexpr int SDMOSI = 23;
    static constexpr int SDSCLK = 22;
    static constexpr int SDCs = 24;

    static constexpr int BuckPGOOD 48;
    static constexpr int BuckEN 44;

    static constexpr int TxCan = 41;
    static constexpr int RxCan = 42;
};


