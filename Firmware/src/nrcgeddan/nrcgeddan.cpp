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

void NRCGeddan::allGotoDesiredAngle(uint16_t desiredAngle, bool unlimited = false) // -15 to 15
{
    if(!unlimited){
        if(desiredAngle > 15){
            desiredAngle = 15;
        } else if(desiredAngle < -15){
            desiredAngle = -15;
        }

    }
    geddanServo1.goto_Angle(desiredAngle + _default_angle1);
    geddanServo2.goto_Angle(desiredAngle + _default_angle2);
    geddanServo3.goto_Angle(desiredAngle + _default_angle3);
}

void NRCGeddan::loadCalibration(){
    
    std::string NVSName = "Srvo1" + std::to_string(_geddanServo1Channel);
    std::string NVSName = "Srvo2" + std::to_string(_geddanServo2Channel);
    std::string NVSName = "Srvo3" + std::to_string(_geddanServo3Channel);
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Servo);
    

    std::vector<uint8_t> calibSerialised = _NVS.loadBytes();
    
    if(calibSerialised.size() == 0){
        return;
    }
    ServoCalibrationPacket calibpacket;
    calibpacket.deserializeBody(calibSerialised);

    setHome(calibpacket.home_angl, calibpacket.home_ang2, calibpacket.home_ang3);
}



void NRCGeddan::update()
{
    switch(currentGeddanState){
        case GeddanState::Startup:
        {
            if (timeFrameCheck(zeroCanards, startSlowSpinLeft))
            {
                allGotoDesiredAngle(0);
            }
            else if (timeFrameCheck(startSlowSpinLeft, startSlowSpinRight))
            {
                allGotoDesiredAngle();
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
void NRCGeddan::gotoHighResAngle(uint16_t angle)
{
    /*Check if angle value is outside of angle limits. Would also have added checking for the min_angle and max_angle but 
    rangemap function already has checking for that so there's no point. */

    if (angle > _angl_lim_max)
    {
        _value = _angl_lim_max;
    }
    else if (angle < _angl_lim_min)
    {
        _value = _angl_lim_min;
    }
    else
    {
        _value = angle; // update new position of actuator
    }

    ledcWrite(_channel, angletocounts((uint16_t)_value));
}

uint16_t NRCRemoteServo::angletocounts(uint16_t angle)
{
    return LIBRRC::rangemap<uint16_t>(angle,_min_angle,_max_angle,_min_counts,_max_counts); 
}

void NRCRemoteServo::reset()
{
    goto_Angle(_default_angle);
}