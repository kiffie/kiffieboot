/*
 * logger_config.h
 * configuration for logger.h
 */

#ifndef __LOGGER_CONFIG_H__
#define __LOGGER_CONFIG_H__

#ifndef DISABLE_LOGGING
    #define LOGLEVEL_USB LOG_INFO
#else
    #define LOGLEVEL_USB LOG_NONE
    #define LOGLEVEL_MAIN LOG_NONE
#endif
#endif
