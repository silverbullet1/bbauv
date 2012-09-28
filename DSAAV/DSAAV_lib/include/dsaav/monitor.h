/**
 * DSAAV Library - Monitor Interface.
 *
 * (c) 2007 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.5 $
 */

#ifndef _DSAAV_MONITOR_H_
#define _DSAAV_MONITOR_H_

// constants
static const int MAX_HEALTH_NAME_LEN = 32;

/**
 * Health Status Codes.
 */
enum Health {
  HS_OFFLINE = 0x00,           //!< Service is offline.
  HS_MALFUNCTION = 0x01,       //!< Service is online but malfunctioning.
  HS_UNAVAILABLE = 0x02,       //!< Service is online but temporarily unavailable.
  HS_HEALTHY = 0x03            //!< Service is online and working fine.
};

/**
 * Severity Level Codes.
 */
enum Severity {
  SL_NONE = 0x00,              //!< No problem.
  SL_WARN = 0x01,              //!< Log a warning regarding the problem.
  SL_ABORT = 0x02,             //!< Abort the mission.
  SL_EMERGENCY = 0x03,         //!< Take emergency action (drop emergency ballast).
  SL_UNKNOWN = 0x04            //!< Unknown severity.
};

/**
 * Health Record.
 */
struct HealthRecord {
  char name[MAX_HEALTH_NAME_LEN];
  Health health;
  double expiry;
  Severity severity[3];
};

/**
 * Health Record Helper Class.
 */
class HealthRecords {
  private:
    List* rec;
  public:
    HealthRecords(void);
    ~HealthRecords(void);
    int count(void);
    String* getName(int i);
    Health getHealth(int i);
    Health getHealth(const char* name);
    Health getHealth(String* name);
    HealthRecord* getHealthRecord(int i);
    HealthRecord* getHealthRecord(const char* name);
    HealthRecord* getHealthRecord(String* name);
    int getHealthRecordNo(const char* name);
    int getHealthRecordNo(String* name);
    void setHealth(int i, Health health, double expiry=0);
    int setHealth(const char* name, Health health, double expiry=0);
    int setHealth(String* name, Health health, double expiry=0);
};

/**
 * Monitor RPC Interface.
 */
class Monitor {
  private:
    MsgSvcAddr* server;
    Rpc* rpc;
    String* myName;
  public:
    Monitor(Rpc* rpc, MsgSvcAddr* addr, const char* name);
    ~Monitor(void);
    Health getHealth(const char* name);
    HealthRecords* getEverybodysHealth(void);
    void setHealth(Health health, float validity);
    void setHealth(Health health, float validity, Severity unavail, Severity malfunc, Severity offline);
    void setHealth(const char* name, Health health, float validity);
    void setHealth(const char* name, Health health, float validity, Severity unavail, Severity malfunc, Severity offline);
    Severity getProblemSeverity(void);
};

#endif
