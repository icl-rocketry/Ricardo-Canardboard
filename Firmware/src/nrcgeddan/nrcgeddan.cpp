#include "nrcgeddan.h"
#include <Arduino.h>
#include <libriccore/commands/commandhandler.h>
#include "config/services_config.h"

void NRCGeddan::setup()
{
    geddanServo1.setup();
    geddanServo2.setup();
    geddanServo3.setup();
    prevLogMessageTime = millis();
}

void NRCGeddan::allGotoCalibratedAngle(uint16_t angle)
{
    geddanServo1.goto_Angle(angle);
    geddanServo2.goto_Angle(angle);
    geddanServo3.goto_Angle(angle);
}

void NRCRemoteServo::loadCalibration(){
    
    std::string NVSName = "Srvo1" + std::to_string(_geddanServo1Channel);
    std::string NVSName = "Srvo2" + std::to_string(_channel);
    std::string NVSName = "Srvo3" + std::to_string(_channel);
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    

    std::vector<uint8_t> calibSerialised = _NVS.loadBytes();
    
    if(calibSerialised.size() == 0){
        return;
    }
    ServoCalibrationPacket calibpacket;
    calibpacket.deserializeBody(calibSerialised);

    setHome(calibpacket.home_angl);
    setAngleLims(calibpacket.min_angl, calibpacket.max_angl);

}


void NRCGeddan::update()
{
    switch(currentGeddanState){
        case GeddanState::Startup:
        {
            if (timeFrameCheck(zeroCanards, startSlowSpinLeft))
            {
                all_goto_angle(90);
            }
            else if (timeFrameCheck(startSlowSpinLeft, startSlowSpinRight))
            {
                all_goto_angle();
            }
            else if (timeFrameCheck(startSlowSpinRight, startSlowSpinRight))
            {
                fuelServo.goto_Angle(90);
            }

            
            geddanServo1.goto_Angle(0);
            geddanServo2.goto_Angle(0);
            geddanServo3.goto_Angle(0);
            break;
        }
        case GeddanState::Shutdown:
        {
            geddanServo1.goto_Angle(0);
            geddanServo2.goto_Angle(0);
            geddanServo3.goto_Angle(0);
            break
        }
        case GeddanState::MotorIdle:
        {

        }
    }
}

bool NRCGeddan::timeFrameCheck(int64_t start_time, int64_t end_time)
{
    if (millis() - ignitionTime > start_time && end_time == -1){
        return true;
    }
    
    else if (millis() - ignitionTime > start_time && millis() - ignitionTime < end_time){
        return true;
    }

    else{
        return false;
    }
}