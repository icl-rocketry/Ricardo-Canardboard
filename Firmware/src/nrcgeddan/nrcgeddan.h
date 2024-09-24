#pragma once

#include "librrc/Remote/nrcremoteactuatorbase.h"
#include "librrc/Remote/nrcremoteservo.h"
#include <librrc/HAL/localpwm.h>
#include <librrc/Helpers/nvsstore.h>
#include <librrc/Helpers/rangemap.h>

#include <libriccore/riccorelogging.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/filtering/movingAvg.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <librrc/Packets/servocalibrationpacket.h>

#include "SiC43x.h"
#include "Sensors/icm_20608.h"
#include "Sensors/sensorStructs.h"
#include "Config/services_config.h"
#include "Loggers/GeddanLogger/geddanlogframe.h"

#include <Arduino.h>


namespace Types
{
    using Servo_t = NRCRemoteServo<LocalPWM>;
    using ServoAdapter_t = RemoteActuatorAdapter<Types::Servo_t>;
    using ServoADPMap_t = std::array<ServoAdapter_t *, 1>;
};


struct GyroReading
{
    float rollRate;
    uint32_t timestamp;

    GyroReading(float rollRate, uint32_t timestamp):
        rollRate(rollRate), 
        timestamp(timestamp)
        {};
};

class NRCGeddan : public NRCRemoteActuatorBase<NRCGeddan>
{

    public:

        NRCGeddan(RnpNetworkManager &networkmanager,
                    SiC43x &buck,
                    ICM_20608 &imu,
                    uint8_t geddanServoGPIO,
                    uint8_t geddanServoChannel
                    ):
            NRCRemoteActuatorBase(networkmanager),
            m_networkmanager(networkmanager),
            m_buck(buck),
            m_imu(imu),
            m_geddanPWM(geddanServoGPIO,geddanServoChannel),
            m_geddanServo(m_geddanPWM,networkmanager,"Srvo0",0,0,1800,500,2500,0,1800), //! All angles x10 for better precision.
            m_geddanServoAdapter(0,m_geddanServo,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            m_movingAvg(100)
            {};
        

        void setup();
        void update();

        

    protected:

        // Networking
        RnpNetworkManager& m_networkmanager;
        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        SiC43x& m_buck;
        ICM_20608& m_imu;

        // Actuators
        LocalPWM m_geddanPWM;
        Types::Servo_t m_geddanServo;
        Types::ServoAdapter_t m_geddanServoAdapter;
        uint8_t geddanServoService = (uint8_t) Services::ID::Servo0;

        // Control
        float m_default_angle;
        void gotoRawAngle(float angle);
        void updateRollRate();
        void gotoCalibratedAngle(float angle);
        void updateTargetRollRate(float targetRollRate);

        void execute_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr);

        // Filtering
        MovingAvg m_movingAvg;

        // Geddan states
        enum class GeddanState : uint8_t
        {
            Locked = 0,
            ConstantRoll = 1,
            WiggleTest = 2,
            Debug = 3, 
            Fun = 4,
        };
        GeddanState m_currentGeddanState = GeddanState::Locked;

        // P Controller
        void resetMovingAverage();

        float error;
        const float m_kp = 0.05;
        SensorStructs::ACCELGYRO_6AXIS_t m_imudata;

        float m_zRollRate;
        float m_targetRollRate = 0;
        float m_avgRollRate;
        float m_servoAngle;


        // Wiggle Test Stuff

        const uint64_t zeroCanards = 0;
        const uint64_t startSlowSpinLeft = 1000;
        const uint64_t zeroCanards2 = 2000;
        const uint64_t startSlowSpinRight = 3000;
        const uint64_t zeroCanards3 = 4000;
        const uint64_t zeroCanards4 = 5000;
        const uint64_t startSpinLeft = 5500;
        const uint64_t startSpinRight = 6000;
        const uint64_t endOfWiggleSeq = 6500  ;
        float lerp(float x, float in_min, float in_max, float out_min, float out_max);
        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);

        uint32_t wiggleTestTime;
        GeddanState previousGeddanState;

        void logReadings();
        uint32_t m_telemetry_log_delta = 5000;
        uint32_t m_prev_telemetry_log_time;
        uint64_t m_prevLogMessageTime;
};