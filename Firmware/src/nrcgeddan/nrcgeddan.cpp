#include "nrcgeddan.h"
#include "geddancalibrationpacket.h"
#include <Arduino.h>
#include <libriccore/commands/commandhandler.h>
#include "config/services_config.h"
#include <librrc/Helpers/nvsstore.h>
#include "librrc/Helpers/rangemap.h"
#include "Loggers/GeddanLogger/geddanlogframe.h"



void NRCGeddan::setup()
{
    geddanServo1.setup();
    geddanServo2.setup();
    geddanServo3.setup();
    prevLogMessageTime = millis();
    loadCalibration();


    previousGeddanState = currentGeddanState;
    wiggleTestTime = millis();
    currentGeddanState = GeddanState::WiggleTest;
}
void NRCGeddan::allGotoCalibratedAngle(float desiredAngle) // -15 to 15
{
    if(desiredAngle > 15){
        desiredAngle = 15;
    } else if(desiredAngle < -15){
        desiredAngle = -15;
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
    
    std::string NVSName = "Canard";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Canard);
    
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
    logReadings();

    // if (this -> _state.flagSet(COMPONENT_STATUS_FLAGS::DISARMED))
    // {
    //     currentGeddanState = GeddanState::HoldZero;
    // }
    
    switch(currentGeddanState){
        case GeddanState::ConstantRoll:
        {
            _imu.update(_imudata);
            _zRollRate = _imudata.gz;            
            
            rollingAverageSum += _zRollRate;
            
            if(!rollingLengthReached)
            {
                
                gyroAverage.push(GyroReading(_zRollRate, millis()));


                rollingAverage = rollingAverageSum / static_cast<float>(rollingAverageCounter);
                if(millis() - rollingAverageStart > rollingAverageDuration)
                {
                    rollingLengthReached = true;
                    rollingAverageLength = rollingAverageCounter;
                }
            } else
            {
                rollingAverageSum -= gyroAverage.pop_push_back(_zRollRate);
                rollingAverage = rollingAverageSum / static_cast<float>(rollingAverageLength);
            }
            
            rollingAverageCounter ++;

            error = _targetRollRate - rollingAverage;

            allGotoCalibratedAngle(- error * _kp);

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
                allGotoRawAngle(lerp(millis() - wiggleTestTime, startSlowSpinLeft, startSlowSpinLeft + 800, 90, 0));
            }
            else if (timeFrameCheck(zeroCanards2, startSlowSpinRight))
            {
                allGotoRawAngle(lerp(millis() - wiggleTestTime, zeroCanards2, zeroCanards2 + 800, 0, 90));
            }
            else if (timeFrameCheck(startSlowSpinRight, zeroCanards3))
            {
                allGotoRawAngle(lerp(millis() - wiggleTestTime, startSlowSpinRight, startSlowSpinRight + 800, 90, 180));
            }
            else if (timeFrameCheck(zeroCanards3, zeroCanards4))
            {
                allGotoRawAngle(lerp(millis() - wiggleTestTime, zeroCanards3, zeroCanards3 + 800, 180, 90));
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
                resetMovingAverage();
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

float NRCGeddan::lerp(float x, float in_min, float in_max, float out_min, float out_max)
{
    float out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    if (out > (out_max > out_min ? out_max : out_min))
    {
        return (out_max > out_min ? out_max : out_min);
    }
    if (out < (out_min < out_max ? out_min : out_max))
    {
        return (out_min < out_max ? out_min : out_max);
    }
    return out;
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
    currentGeddanState = GeddanState::ConstantRoll;
    SimpleCommandPacket execute_command(*packetptr);
    _targetRollRate = execute_command.arg;
}
void NRCGeddan::calibrate_impl(packetptr_t packetptr)
{
    GeddanCalibrationPacket calibrate_comm(*packetptr);

    std::vector<uint8_t> serialisedvect = packetptr->getBody();

    std::string NVSName = "Canard";
    NVSStore _NVS(NVSName, NVSStore::calibrationType::Canard);
    
    _NVS.saveBytes(serialisedvect);
    
    setHome(calibrate_comm.home_ang1, calibrate_comm.home_ang2, calibrate_comm.home_ang3);

}
void NRCGeddan::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr){
    SimpleCommandPacket command_packet(*packetptr);
    switch(static_cast<uint8_t>(commandID))
    {
        case 6: // Wiggle test
        {
            previousGeddanState = currentGeddanState;
            wiggleTestTime = millis();
            currentGeddanState = GeddanState::WiggleTest;
            break;
        }
        case 7: // Debug go to calibrated angle
        {
            if(currentGeddanState == GeddanState::Debug){
                allGotoCalibratedAngle(command_packet.arg);
            }

            break;
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
        case 9:
        {
            currentGeddanState = static_cast<GeddanState>(command_packet.arg);
            resetMovingAverage();
        }
        default:
        {
            NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID,std::move(packetptr));
            break;
        }
    }
}
void NRCGeddan::resetMovingAverage()
{
    rollingLengthReached = false;
    rollingAverageCounter = 0;
    rollingAverageStart = millis();
}

void NRCGeddan::logReadings()
{
    if (micros() - prev_telemetry_log_time > telemetry_log_delta)
    {
        GeddanLogFrame logframe;

        logframe.zRollRate = _zRollRate;
        logframe.movingAverage = rollingAverage;

        logframe.timestamp = micros();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::GEDDAN>(logframe);

        prev_telemetry_log_time = micros();
    }
}