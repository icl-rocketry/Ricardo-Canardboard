
#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

class GeddanCalibrationPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &GeddanCalibrationPacket::command,
                &GeddanCalibrationPacket::home_ang1,
                &GeddanCalibrationPacket::home_ang2,
                &GeddanCalibrationPacket::home_ang3
            );

            return ret;
        }
        
    public:
        ~GeddanCalibrationPacket();

        GeddanCalibrationPacket();
        /**
         * @brief Deserialize Packet
         * 
         * @param data 
         */
        GeddanCalibrationPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        void deserializeBody(std::vector<uint8_t>& buf);
        
        uint8_t command;
        uint16_t home_ang1;
        uint16_t home_ang2;
        uint16_t home_ang3;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


