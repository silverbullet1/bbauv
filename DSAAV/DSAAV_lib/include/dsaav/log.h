/**
 * DSAAV Library - Logging Service Interface.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.6 $
 */

#ifndef _DSAAV_LOG_H_
#define _DSAAV_LOG_H_

/**
 * Log levels.
 */
enum LogLevel {
  LOG_NONE,             //!< Log nothing.
  LOG_ERROR,            //!< Log errors only.
  LOG_WARNING,          //!< Log errors and warnings.
  LOG_INFO,             //!< Log errors, warnings and info messages.
  LOG_DATA,             //!< Log errors, warnings, info messages and data.
  LOG_DEBUG,            //!< Log everything.
  LOG_INVALID           //!< Invalid log level, internal use only.
};

/**
 * Abstract logging interface.
 */
class Logger {
  private:
    String* name;
    LogLevel logLevel;
  protected:
    Logger(void);
    void setName(const char* name);
  public:
    virtual ~Logger(void);
    void setLogLevel(LogLevel level);
    void debug(const char* fmt, ...);
    void data(const char* fmt, ...);
    void info(const char* fmt, ...);
    void warning(const char* fmt, ...);
    void error(const char* fmt, ...);
    void abort(const char* fmt, ...);
    
    /**
     * Log entry using appropriate subclass method implementation.
     *
     * \param level     Log level to log this entry at.
     * \param module    Name of calling module.
     * \param entry     ASCIIZ string to be logged.
     */    
    virtual void log(LogLevel level, const char* module, const char* entry) = 0;
};

/**
 * Logs entries to a file.
 */
class FileLogger : public Logger {
  public:
    FileLogger(const char* name);
    ~FileLogger(void);
    void log(LogLevel level, const char* module, const char* entry);
};

/**
 * Logs entries to a logging server using RPC.
 */
class RemoteLogger : public Logger {
  private:
    MsgSvcAddr* server;
    Rpc* rpc;
  public:
    RemoteLogger(Rpc* rpc, MsgSvcAddr* server, const char* name);
    ~RemoteLogger(void);
    void log(LogLevel level, const char* module, const char* entry);
};

#endif
