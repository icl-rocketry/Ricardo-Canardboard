#include "system.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "Commands/commands.h"

#include "States/idle.h"
static constexpr int SPI_BUS_NUM = 0;
static constexpr int SDSPI_BUS_NUM = 1;

System::System():   RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
                    spi(SPI_BUS_NUM),
                    sdspi(SDSPI_BUS_NUM),
                    Buck(PinMap::BuckPGOOD, PinMap::BuckEN, 0, 1, PinMap::BuckOutputV, 34000, 3000),
                    IMU(spi, systemstatus, PinMap::ImuCs),
                    canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
                    primarysd(SDSPI,PinMap::SDCs,SD_SCK_MHZ(20),false,&systemstatus),
                    Geddan(networkmanager,Buck,IMU,PinMap::ServoPWM0,0)
                    {};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
    
    std::array<uint8_t,3> axesOrder{0,1,2};
    std::array<bool,3> axesFlip{0,0,0};

    setupPins();
    setupSPI();

    IMU.setup(axesOrder,axesFlip);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    //any other setup goes here

    canbus.setup();
    Buck.setup();
    Geddan.setup();

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});
    networkmanager.addInterface(&canbus);

    uint8_t geddanservice = static_cast<uint8_t>(Services::ID::Geddan);
    networkmanager.registerService(geddanservice,Geddan.getThisNetworkCallback());

    primarysd.setup();
    initializeLoggers();
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
    sdspi.setFrequency(50000000);
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

void System::initializeLoggers()
{
    // check if sd card is mounted
    if (primarysd.getState() != StoreBase::STATE::NOMINAL)
    {
        loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(nullptr, networkmanager);

        return;
    }

    // open log files
    // get unique directory for logs
    std::string log_directory_path = primarysd.generateUniquePath(log_path, "");
    // make new directory
    primarysd.mkdir(log_directory_path);

    std::unique_ptr<WrappedFile> syslogfile = primarysd.open(log_directory_path + "/syslog.txt", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END));
    std::unique_ptr<WrappedFile> geddanlogfile = primarysd.open(log_directory_path + "/geddanlog.txt", static_cast<FILE_MODE>(O_WRITE | O_CREAT | O_AT_END),50);

    // intialize sys logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(std::move(syslogfile), networkmanager);

    // initialize telemetry logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::GEDDAN>().initialize(std::move(geddanlogfile),[](std::string_view msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);});
}