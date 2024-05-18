#pragma once
/**
 * @file sensors.h
 * @author Kiran de Silva
 * @brief Manages sensor suite on avionics.
 * TODO:
 * Threadsafe
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <SPI.h>
#include <Wire.h>
#include <memory>
#include <functional>
#include <ArduinoJson.h>


#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>


#include "Config/types.h"
#include "sensorStructs.h"

#include "icm_20608.h"
#include "mmc5983ma.h"

class Sensors
{
public:
    Sensors(SPIClass &spi, TwoWire &I2C, Types::CoreTypes::SystemStatus_t &systemstatus);

    void setup();
    void update();

    /**
     * @brief Get the Raw Sensor Data
     *
     * @return const SensorStructs::raw_measurements_t&
     */
    const SensorStructs::raw_measurements_t &getData();

    // Sensor Calibration Functions
    void calibrateAccelGyro();
    void calibrateHighGAccel();
    void calibrateMag(MagCalibrationParameters magcal);
    void calibrateBaro();

    std::function<void(std::unique_ptr<RnpPacketSerialized>)> getHitlCallback();

private:
    SensorStructs::raw_measurements_t sensors_raw;
    Types::CoreTypes::SystemStatus_t& _systemstatus;

    ICM_20608 accelgyro;
    MMC5983MA mag;

    /**
     * @brief Handle fake sensor data packets from hardware in the loop service
     * 
     * @param packet_ptr 
     */
    void hitlHandler(std::unique_ptr<RnpPacketSerialized> packet_ptr);
    void hitlCommandHandler(RnpPacketSerialized& packet);
    bool _hitlEnabled;

    void hitlUpdateSensorError(uint8_t sensor_state,SYSTEM_FLAG flag);

};
