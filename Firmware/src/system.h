#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"

#include "Commands/commands.h"
#include <libriccore/networkinterfaces/can/canbus.h>
#include <SPI.h>

#include "SiC43x.h"
#include "sensors/icm_20608.h"
#include "nrcgeddan/nrcgeddan.h"

class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SPIClass spi;
        SPIClass sdspi;

        SiC43x Buck;
        ICM_20608 IMU;

        CanBus<SYSTEM_FLAG> canbus;

    private:
        void setupSPI();
        void setupPins();

        NRCGeddan Geddan;
};