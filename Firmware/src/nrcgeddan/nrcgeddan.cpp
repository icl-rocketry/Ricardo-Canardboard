#include "nrcgeddan.h"
#include "geddancalibrationpacket.h"
#include <Arduino.h>
#include <libriccore/commands/commandhandler.h>
#include "config/services_config.h"
#include <librrc/Helpers/nvsstore.h>

void NRCGeddan::setup()
{
    geddanServo1.setup();
    geddanServo2.setup();
    geddanServo3.setup();
    prevLogMessageTime = millis();
}

void NRCGeddan::allGotoDesiredAngle(float desiredAngle, bool unlimited = false) // -15 to 15
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
    GeddanCalibrationPacket calibpacket;
    calibpacket.deserializeBody(calibSerialised);

    setHome(calibpacket.home_ang1, calibpacket.home_ang2, calibpacket.home_ang3);
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
                //  TODO: Make this go slower
                geddanServo1.goto_AngleHighRes(0);
                geddanServo2.goto_AngleHighRes(0);
                geddanServo3.goto_AngleHighRes(0);
            }
            else if (timeFrameCheck(startSlowSpinRight, startSlowSpinRight))
            {
                //  TODO: Make this go slower
                geddanServo1.goto_AngleHighRes(180);
                geddanServo2.goto_AngleHighRes(180);
                geddanServo3.goto_AngleHighRes(180);
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

void NRCGeddan::execute_impl(packetptr_t packetptr) //Set it to roll rate, mili radians per second
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1:
    {
        if (currentGeddanState != GeddanState::)
        {
            break;
        }
        currentEngineState = EngineState::Ignition;
        ignitionTime = millis();
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ignition");
        break;
    }
    case 2:
    {
        currentEngineState = EngineState::ShutDown;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3:
    {
        if (currentEngineState != EngineState::ShutDown)
        {
            break;
        }
        currentEngineState = EngineState::Debug;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    }
}