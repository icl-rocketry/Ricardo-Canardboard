#include "Config/loggerhandler_config.h"

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include <libriccore/logging/loggers/syslogger.h>

#include "Loggers/GeddanLogger/geddanlogger.h"
#include "Loggers/TelemetryLogger/telemetrylogger.h"

std::tuple<SysLogger,TelemetryLogger,CoutLogger,GeddanLogger> RicCoreLoggingConfig::logger_list =
{
    SysLogger(),
    TelemetryLogger(),
    CoutLogger("COUT_LOG"),
    GeddanLogger()
};