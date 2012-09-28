/**
 * STARFISH Core Servers.
 *
 * DSAAV Implementation.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.15 $
 */
 
#ifndef _DSAAV_CORE_SERVERS_H_
#define _DSAAV_CORE_SERVERS_H_

#include <time.h>
#include <sys/stat.h>
#include <dsaav/dsaav.h>

/**
 * Configuration server.
 *
 * The configuration file at the server consists of multiple sections.
 * Each section is denoted using square brackets (e.g. [Section 1]).
 * Each section contains multiple entries, each consisting of a key
 * and value separated by an equals sign. Comments can be inserted
 * on a line after a hash symbol.
 *
 * An example configuration file is shown below:
 * \code
 * # An example configuration file
 *
 * [SomeSection]
 * Entry1 = Some Value     # string value
 * SomeNumber = 21         # numeric value
 *
 * [AnotherSection]
 * AnotherEntry = 19.32
 * RefEntry = Item$(SomeSection.SomeNumber)
 * Address = $(MyAddress):0
 * \endcode
 */

class ConfigurationServer : public RpcService {
  private:
    String* cfgfile;
    List* sections;
    String* addr;
#ifdef _OSX
    struct timespec mtime;
#else
    time_t mtime;
#endif
    double lastCheck, lastTouch;
    void load(void);
    void loadFile(FILE* fp, int persist);
    void unload(void);
    String* resolveVars(String* s, List* section);
    void put(const char* section, const char* key, const char* value);
    void remove(const char* section, const char* key);
    void store(void);
  public:
    ConfigurationServer(const char* myaddr);
    ConfigurationServer(const char* filename, const char* myaddr);
    ~ConfigurationServer(void);
    String* getParameter(const char* section, const char* key);
    ParamSet* service(ParamSet* params);
    void tick(void);
};

/**
 * RPC logging server.
 */

class LoggingServer : public RpcService {
  private:
    FileLogger* log;
    String* logFilename; //used when there is specified filename assigned in starfish.rc
    String* filename;
    int autoFilename, autoDTS;
    void open(void);
    ParamSet* processSpecialCommand(String* cmd);
  public:
    LoggingServer(void);
    LoggingServer(const char* filename, int autoDTS = 1);
    ~LoggingServer(void);
    ParamSet* service(ParamSet* params);
};

/**
 * Monitor server.
 *
 * The monitor server uses configuration file entries to determine
 * the modules to monitor. An example configuration section is shown
 * below:
 *
 * \code
 * [Monitor]
 * Server = 123456789abc:0          # server address
 * Items = 3                        # number of items to monitor
 * Item:1 = Elevators,0,3,3,3       # Each entry has the following format:
 * Item:2 = Depth,5,1,2,2           #  name,poll,unavail,malfunc,offline
 * Item:3 = Sidescan,0,1,1,1
 * \endcode
 *
 * Each item to monitor is specified by a name, which in turn allows
 * the server to be looked up from the configuration. If the poll value
 * in the entry is 0, the monitor server expects the module to report
 * its status regularly using the RPC_HEALTH_SET call. For non-zero
 * values of poll, the monitor server makes RPC_HEALTH_GET calls
 * to the service at a nominal interval specified by the poll value in
 * seconds. The unavail, malfunc and offline flags in each entry specify
 * the severity level for each type of failure of the item. A
 * severity level of 0 implies "doesn't matter", 1 implies "warning",
 * 2 implies "abort mission" and 3 implies "emergency ballast drop". This
 * should be monitored by the safety officer to take the appropriate
 * action.
 */

class MonitorServer : public RpcService {
  private:
    HealthRecords* db;
    List* polls;
    Rpc* rpc;
    int pollMargin;
  protected:
    Severity getProblemSeverity(void);
    void init(Rpc* rpc, MsgSvcAddr* addr);
    Severity attr2severity(const char* s, Severity d);
    Health attr2health(const char* s, Health d);
    int attr2int(const char* s, int d);
  public:
    MonitorServer(Rpc* rpc);
    MonitorServer(Rpc* rpc, MsgSvcAddr* cfgSvr);
    ~MonitorServer(void);
    ParamSet* service(ParamSet* params);
    void tick(void);
};

#endif
