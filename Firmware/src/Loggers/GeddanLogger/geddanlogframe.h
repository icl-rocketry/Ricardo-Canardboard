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
            &GeddanLogFrame::servoAngle,
            &GeddanLogFrame::geddanState,
            &GeddanLogFrame::armed,
            &GeddanLogFrame::timestamp
        );
        return ret;
    }

public:
    //gps
    float zRollRate;
    float movingAverage;
    float servoAngle; //Bad to in single line
    uint8_t geddanState;
    bool armed;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
