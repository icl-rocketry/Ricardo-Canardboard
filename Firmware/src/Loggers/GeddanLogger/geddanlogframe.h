#pragma once

#include <librnp/rnp_serializer.h>
#include <unistd.h>

class GeddanLogFrame{
private:  
    static constexpr auto getSerializer()
    {
        auto ret = RnpSerializer(
            &GeddanLogFrame::zRollRate,
            &GeddanLogFrame::movingAverage
        );
        return ret;
    }

public:
    //gps
    float zRollRate, movingAverage;

    uint64_t timestamp;

    std::string stringify()const{
        return getSerializer().stringify(*this) + "\n";
    };

};
