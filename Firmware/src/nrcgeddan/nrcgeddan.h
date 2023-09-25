#pragma once

#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <librrc/packets/servocalibrationpacket.h>
#include "SiC43x.h"

class NRCGeddan : public NRCRemoteActuatorBase<NRCGeddan>
{

    public:

        NRCGeddan(RnpNetworkManager &networkmanager,
                    SiC43x &buck,
                    uint8_t geddanServo1GPIO,
                    uint8_t geddanServo2GPIO,
                    uint8_t geddanServo3GPIO,
                    uint8_t geddanServo1Channel,
                    uint8_t geddanServo2Channel,
                    uint8_t geddanServo3Channel,
                    uint8_t address
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _buck(buck),
            _networkmanager(networkmanager),
            _geddanServo1GPIO(geddanServo1GPIO),
            _geddanServo2GPIO(geddanServo2GPIO),
            _geddanServo3GPIO(geddanServo3GPIO),
            _geddanServo1Channel(geddanServo1Channel),
            _geddanServo2Channel(geddanServo2Channel),
            _geddanServo3Channel(geddanServo3Channel),
            _address(address),
            geddanServo1(geddanServo1GPIO,geddanServo1Channel,networkmanager),
            geddanServo2(geddanServo2GPIO,geddanServo2Channel,networkmanager),
            geddanServo3(geddanServo3GPIO,geddanServo3Channel,networkmanager)
            {};
        

        void setup();
        void update();
        void gotoHighResAngle(uint16_t angle);
        void allGotoDesiredAngle(float angle, bool unlimited = false);
        void updateTargetRollRate(float targetRollRate);
        

    protected:

        RnpNetworkManager& _networkmanager;
        const uint8_t _geddanServo1GPIO;
        const uint8_t _geddanServo2GPIO;
        const uint8_t _geddanServo3GPIO;
        const uint8_t _geddanServo1Channel;
        const uint8_t _geddanServo2Channel;
        const uint8_t _geddanServo3Channel;
        const uint8_t _address;
        SiC43x& _buck;

        void loadCalibration();
        uint16_t _default_angle1;
        uint16_t _default_angle2;
        uint16_t _default_angle3;

        void setHome(uint16_t homeangle1, u_int16_t homeangle2, u_int16_t homeangle3){
            _default_angle1 = homeangle1;
            _default_angle2 = homeangle2;
            _default_angle3 = homeangle3;
        };

        NRCRemoteServo geddanServo1;
        NRCRemoteServo geddanServo2;    
        NRCRemoteServo geddanServo3;    

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        
        void execute_impl(packetptr_t packetptr);
        //void arm_impl(packetptr_t packetptr);
        //void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr);

        enum class GeddanState : uint8_t
        {
            Startup = 0,
            Shutdown = 1,
            ConstantRoll = 2,
            Default = 3, //Hold zero angle
            Debug = 4,
            Fun = 5
        };

        bool default_called = false;
        bool shutdown_called = false;

        GeddanState currentGeddanState = GeddanState::Default;


        float _zRollRate;
        float _targetRollRate;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();
        float demandedFuelP();
        
        void firePyro(uint32_t duration);

        //Ignition sequence timings from moment ignition command received
        const uint64_t zeroCanards = 0;
        const uint64_t startSlowSpinLeft = 500;
        const uint64_t startSlowSpinRight = 1500;
        const uint64_t zeroCanards2 = 2500;
        const uint64_t startSpinLeft = 5000;
        const uint64_t startSpinRight = 5500;
        const uint64_t endOfIgnitionSeq = 6000;
        long prevprint = 0;

        float error;
        const float Kp = 0.5;
        uint16_t currFuelServoAngle;
        uint16_t fuelServoDemandAngle;
        const uint16_t fuelServoPreAngle = 60;
        const uint16_t oxServoPreAngle = 90;

        uint64_t lastTimeFuelPUpdate;
        uint64_t lastTimeChamberPUpdate;

        const uint64_t pressureUpdateTimeLim = 1000;

        uint64_t prevLogMessageTime;

        uint64_t prevFiring = 0;
};