#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"

#include "Commands/commands.h"
#include <libriccore/networkinterfaces/can/canbus.h>
#include <SPI.h>
#include <Wire.h>

#include "Sensors/sensors.h"
#include "Sensors/estimator.h"

#include "SiC43x.h"
#include "sensors/icm_20608.h"
#include "nrcgeddan/nrcgeddan.h"
#include "Storage/sdfat_store.h"

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SPIClass spi;
        SPIClass sdspi;
        TwoWire I2C;

        SiC43x Buck;
        ICM_20608 IMU;

        CanBus<SYSTEM_FLAG> canbus;

        Sensors sensors;
        Estimator estimator;

        SPIClass SDSPI; //SPI for the SD card
        SPIClass SNSRSPI; //SPI for the sensors

        

        SdFat_Store primarysd;

    private:
        void setupSPI();
        void setupPins();
        void initializeLoggers();

        const std::string log_path = "/Logs";

        NRCGeddan Geddan;
};