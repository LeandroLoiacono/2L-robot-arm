#ifndef LOGGER_H_
#define LOGGER_H_

#include <Arduino.h>

#define LOG_ERROR 0
#define LOG_INFO 1
#define LOG_DEBUG 2

#ifndef LOG_LEVEL
#define LOG_LEVEL 0 //DEFAULT VALUE OVERWRITE IT IN config.h
#endif

class Logger {
  public:
    static void Logger::log(String message, int level);
    static void Logger::logINFO(String message);
    static void Logger::logERROR(String message);
    static void Logger::logDEBUG(String message);
};
#endif
