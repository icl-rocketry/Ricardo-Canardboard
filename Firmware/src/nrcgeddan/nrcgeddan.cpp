#include "nrcgeddan.h"

void NRCGeddan::setup()
{
    m_geddanServo.setup();
    m_default_angle = m_geddanServo.getValue();     // add to update in case it gets recalibrated.

    _networkmanager.registerService(geddanServoService,m_geddanServo.getThisNetworkCallback());

    m_prevLogMessageTime = millis();
}

void NRCGeddan::gotoCalibratedAngle(float desiredAngle) // -15 to 15
{
    if(desiredAngle > 14.0f){
        desiredAngle = 14.0f;
    } else if(desiredAngle < -14.0f){
        desiredAngle = -14.0f;
    }

    m_geddanServoAdapter.execute(static_cast<uint32_t>(m_default_angle + desiredAngle*10.0f));
}

void NRCGeddan::gotoRawAngle(float angle)
{
    m_geddanServoAdapter.execute(static_cast<uint32_t>(m_default_angle + angle*10.0f));
}

void NRCGeddan::updateRollRate(){
    m_imu.update(m_imudata);
    m_zRollRate = m_imudata.gz;            
    m_movingAvg.update(m_zRollRate); 
    m_avgRollRate = m_movingAvg.getAvg();
}


void NRCGeddan::update()
{
    updateRollRate();


    if (this -> _state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED))
    {
        if (!m_geddanServoAdapter.getState().flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED)){
            m_geddanServoAdapter.disarm();   // servo state to match nrcgeddan
            m_buck.setEN(false);             // disable buck
        }

        if (static_cast<int32_t>(m_default_angle) != m_geddanServo.getValue()){
            m_default_angle = m_geddanServo.getValue();
        }

    }

    if (this -> _state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL))
    {
        if (!m_geddanServoAdapter.getState().flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL)){
            m_geddanServoAdapter.arm(0);   // servo state to match nrcgeddan
            m_buck.setEN(true);             // enable buck
        }

    }

    
    switch(m_currentGeddanState){
        case GeddanState::Locked:
        {
            break;  // if geddan is armed, so is the servo. if the servo is armed its gonna hold zero (locked position) by default
        }
        case GeddanState::ConstantRoll:
        {
            error = m_targetRollRate - m_avgRollRate;
            gotoCalibratedAngle(- error * m_kp);
            break;
        }
        case GeddanState::WiggleTest:
        {
            if (timeFrameCheck(zeroCanards, startSpinLeftHard))
            {
                gotoCalibratedAngle(0);
            }
            else if (timeFrameCheck(startSpinLeftHard, startSpinRightHard))
            {
                gotoRawAngle(-90);
            }
            else if (timeFrameCheck(startSpinRightHard, startSpinLeft))
            {
                gotoRawAngle(90);
            }
            else if (timeFrameCheck(startSpinLeft, startSpinRight)){
                gotoCalibratedAngle(-15);
            }
            else if (timeFrameCheck(startSpinRight, endOfWiggleSeq)){
                gotoCalibratedAngle(15);
            }
            else if (timeFrameCheck(endOfWiggleSeq))
            {
                gotoCalibratedAngle(0);
                resetMovingAverage();
                m_currentGeddanState = previousGeddanState;
                
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
        default:
        {
            break;
        }
    }
    
    logReadings();

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
    m_currentGeddanState = GeddanState::ConstantRoll;
    SimpleCommandPacket execute_command(*packetptr);
    m_targetRollRate = execute_command.arg;
}

void NRCGeddan::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID,packetptr_t packetptr){
    SimpleCommandPacket command_packet(*packetptr);
    switch(static_cast<uint8_t>(commandID))
    {
        case 6: // Wiggle test
        {
            previousGeddanState = m_currentGeddanState;
            wiggleTestTime = millis();
            m_currentGeddanState = GeddanState::WiggleTest;
            break;
        }
        case 7: // Debug go to calibrated angle
        {
            if(m_currentGeddanState == GeddanState::Debug){
                gotoCalibratedAngle(command_packet.arg);
            }

            break;
        }
        case 8: // Debug go to raw angle
        {
            if(m_currentGeddanState == GeddanState::Debug){
                gotoRawAngle(command_packet.arg);
                break;
            }
            else{
                break;
            }
        }
        case 9:
        {
            m_currentGeddanState = static_cast<GeddanState>(command_packet.arg);
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
    // m_gyroBuf = GyroBuf();
}

void NRCGeddan::logReadings()
{
    if (micros() - m_prev_telemetry_log_time > m_telemetry_log_delta)
    {
        GeddanLogFrame logframe;

        logframe.zRollRate = m_zRollRate;
        logframe.movingAverage = m_avgRollRate;
        logframe.servoAngle = m_geddanServoAdapter.getValue();
        logframe.geddanState = static_cast<uint8_t>(m_currentGeddanState);
        logframe.armed = this -> _state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED);

        logframe.timestamp = micros();

        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::GEDDAN>(logframe);

        m_prev_telemetry_log_time = micros();
    }
}