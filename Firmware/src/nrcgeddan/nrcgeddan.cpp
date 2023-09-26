#include "nrcgeddan.h"
#include "geddancalibrationpacket.h"
#include <Arduino.h>
#include <libriccore/commands/commandhandler.h>
#include "config/services_config.h"
#include <librrc/Helpers/nvsstore.h>
#include "librrc/Helpers/rangemap.h"


void NRCGeddan::setup()
{
    
    geddanServo1.setup();
    geddanServo2.setup();
    geddanServo3.setup();
    prevLogMessageTime = millis();
}
void NRCGeddan::allGotoCalibratedAngle(float desiredAngle, bool unlimited) // -15 to 15
{
    if(!unlimited){
        if(desiredAngle > 15){
            desiredAngle = 15;
        } else if(desiredAngle < -15){
            desiredAngle = -15;
        }
    }
    geddanServo1.goto_AngleHighRes(desiredAngle + _default_angle1);
    geddanServo2.goto_AngleHighRes(desiredAngle + _default_angle2);
    geddanServo3.goto_AngleHighRes(desiredAngle + _default_angle3);
}
void NRCGeddan::allGotoRawAngle(float angle)
{
    geddanServo1.goto_AngleHighRes(angle);
    geddanServo2.goto_AngleHighRes(angle);
    geddanServo3.goto_AngleHighRes(angle);
}

void NRCGeddan::loadCalibration(){
    
    std::string NVSName = "Srvo1" + std::to_string(_geddanServo1Channel);
    //std::string NVSName = "Srvo2" + std::to_string(_geddanServo2Channel);
    //std::string NVSName = "Srvo3" + std::to_string(_geddanServo3Channel);
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
        case GeddanState::ConstantRoll:
        {
            _imu.update(imustats);
            _zRollRate = imustats.gz;


            

            error = _targetRollRate - _zRollRate;
            allGotoCalibratedAngle(- error * Kp);
            
            break;
        }
        case GeddanState::HoldZero:
        {
            allGotoCalibratedAngle(0);
            break;
        }
        case GeddanState::WiggleTest:
        {
            if (timeFrameCheck(zeroCanards, startSlowSpinLeft))
            {
                allGotoRawAngle(90);
            }
            else if (timeFrameCheck(startSlowSpinLeft, zeroCanards2))
            {
                allGotoRawAngle(LIBRRC::rangemap<float>(millis() - wiggleTestTime, startSlowSpinLeft, startSlowSpinLeft + 800, 90, 0));
            }
            else if (timeFrameCheck(zeroCanards2, startSlowSpinRight))
            {
                allGotoRawAngle(LIBRRC::rangemap<float>(millis() - wiggleTestTime, zeroCanards2, zeroCanards2 + 800, 0, 90));
            }
            else if (timeFrameCheck(startSlowSpinRight, zeroCanards3))
            {
                allGotoRawAngle(LIBRRC::rangemap<float>(millis() - wiggleTestTime, startSlowSpinRight, startSlowSpinLeft + 800, 90, 180));
            }
            else if (timeFrameCheck(zeroCanards3, zeroCanards4))
            {
                allGotoRawAngle(LIBRRC::rangemap<float>(millis() - wiggleTestTime, zeroCanards3, zeroCanards3 + 800, 180, 90));
            }
            else if (timeFrameCheck(zeroCanards4, startSpinLeft))
            {
                allGotoCalibratedAngle(0);
            }
            else if (timeFrameCheck(startSpinLeft, startSpinRight))
            {
                allGotoCalibratedAngle(-15);
            }
            else if (timeFrameCheck(startSpinRight, endOfWiggleSeq))
            {
                allGotoCalibratedAngle(15);
            }
            else if (timeFrameCheck(endOfWiggleSeq))
            {
                allGotoCalibratedAngle(0);
                currentGeddanState = previousGeddanState;
            }
            break;
        }
        case GeddanState::Debug:
        {
            break;
        }
        case GeddanState::Fun:
        {
            break;
        }
    }
}

bool NRCGeddan::timeFrameCheck(int64_t start_time, int64_t end_time)
{
    if (millis() - wiggleTestTime > start_time && end_time == -1){
        return true;
    }
    
    else if (millis() - wiggleTestTime > start_time && millis() - wiggleTestTime < end_time){
        return true;
    }

    else{
        return false;
    }
}

void NRCGeddan::execute_impl(packetptr_t packetptr) //Set it to roll rate, mili radians per second
{
    SimpleCommandPacket execute_command(*packetptr);
    _targetRollRate = execute_command.arg;
}

void NRCGeddan::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr){
    SimpleCommandPacket command_packet(*packetptr);
    switch(static_cast<uint8_t>(commandID))
    {
        case 5:
        {
            currentGeddanState = static_cast<GeddanState>(command_packet.arg);
        }
        case 7: // Debug go to calibrated angle
        {
            if(currentGeddanState == GeddanState::Debug){
                allGotoCalibratedAngle(command_packet.arg);
                break;
            }
            else{
                break;
            }
        }
        case 8: // Debug go to raw angle
        {
            if(currentGeddanState == GeddanState::Debug){
                allGotoRawAngle(command_packet.arg);
                break;
            }
            else{
                break;
            }
        }
        case 6: // Wiggle test
        {
            previousGeddanState = currentGeddanState;
            currentGeddanState = GeddanState::WiggleTest;
            wiggleTestTime = millis();
        }
        default:
        {
            NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID,std::move(packetptr));
            break;
        }
    }
}