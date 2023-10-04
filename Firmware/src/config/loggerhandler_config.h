#pragma once

#include <tuple>

#include <libriccore/logging/loggers/coutlogger.h>
#include <libriccore/logging/loggers/rnpmessagelogger.h>
#include <libriccore/logging/loggers/syslogger.h>
#include "Loggers/GeddanLogger/geddanlogger.h"

#include "Loggers/TelemetryLogger/telemetrylogger.h"


namespace RicCoreLoggingConfig
{
    enum class LOGGERS
    {
        SYS, // default system logging
        TELEMETRY,
        COUT, // cout logging
        GEDDAN
    };

    extern std::tuple<SysLogger,TelemetryLogger,CoutLogger,GeddanLogger> logger_list;
}; 


