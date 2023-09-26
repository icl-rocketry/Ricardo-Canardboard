#include "system.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "config/services_config.h"

#include "Commands/commands.h"

#include "States/idle.h"


System::System():   RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
                    Buck(PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 34000, 3000),
                    IMU(spi, systemstatus, PinMap::ImuCs),
                    canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
                    Geddan(networkmanager,Buck,IMU,PinMap::ServoPWM1,0,PinMap::ServoPWM2,1,PinMap::ServoPWM3,2,networkmanager.getAddress()){};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
    
    setupPins();
    setupSPI();


    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    //any other setup goes here

    canbus.setup();
    Buck.setup();
    uint8_t geddanservice = static_cast<uint8_t>(Services::ID::Geddan);
    networkmanager.registerService(geddanservice,Geddan.getThisNetworkCallback());
};


void System::systemUpdate(){
    Buck.update();
    Geddan.update();
};

void System::setupSPI(){
    spi.begin(PinMap::SCLK,PinMap::MISO,PinMap::MOSI);
    spi.setFrequency(1000000);
    spi.setBitOrder(MSBFIRST);
    spi.setDataMode(SPI_MODE0);

    sdspi.begin(PinMap::SDSCLK,PinMap::SDMISO,PinMap::SDMOSI);
    sdspi.setFrequency(8000000);
    sdspi.setBitOrder(MSBFIRST);
    sdspi.setDataMode(SPI_MODE0);
}

void System::setupPins(){
    pinMode(PinMap::ImuCs, OUTPUT);
    pinMode(PinMap::MagCs, OUTPUT);
    pinMode(PinMap::SDCs, OUTPUT);

    digitalWrite(PinMap::ImuCs, HIGH);
    digitalWrite(PinMap::MagCs, HIGH);
    digitalWrite(PinMap::SDCs, HIGH);

};