#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class GeddanLogFrame{
private:  
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &GeddanLogFrame::zRollRate,
            &GeddanLogFrame::movingAverage,
            &GeddanLogFrame::servo1Angle,
            &GeddanLogFrame::servo2Angle,
            &GeddanLogFrame::servo3Angle,
            &GeddanLogFrame::geddanState,
            &GeddanLogFrame::armed,
            &GeddanLogFrame::timestamp
        );
        return ret;
    }

public:
    //gps
    float zRollRate, movingAverage, servo1Angle, servo2Angle, servo3Angle; //Bad to in single line
    uint8_t geddanState;
    bool armed;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
