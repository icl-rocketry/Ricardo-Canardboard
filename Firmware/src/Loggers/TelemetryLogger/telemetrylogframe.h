#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class TelemetryLogframe{
private:  
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &TelemetryLogframe::ch0sens,
            &TelemetryLogframe::ch1sens,
            &TelemetryLogframe::ch2sens,
            &TelemetryLogframe::ch3sens,
            &TelemetryLogframe::ch4sens,
            &TelemetryLogframe::ch5sens,
            &TelemetryLogframe::ch6sens,
            &TelemetryLogframe::ch7sens,
            &TelemetryLogframe::ch8sens,
            &TelemetryLogframe::ch9sens,
            &TelemetryLogframe::ch10sens,
            &TelemetryLogframe::ch11sens,
            &TelemetryLogframe::temp0,
            &TelemetryLogframe::temp1,
            &TelemetryLogframe::temp2,
            &TelemetryLogframe::temp3,
            &TelemetryLogframe::timestamp
            
           

        );
        return ret;
    }

public:
    //gps
    float ch0sens,ch1sens,ch2sens,ch3sens,ch4sens,ch5sens;
    float ch6sens,ch7sens,ch8sens,ch9sens,ch10sens,ch11sens;

    float temp0,temp1,temp2,temp3;

    
    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
