/**
 * DSAAV Library - Configuration Service Interface.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.8 $
 */

#ifndef _DSAAV_CONFIG_H_
#define _DSAAV_CONFIG_H_

/**
 * Configuration service RPC interface.
 */
class Configuration {
  private:
    String* section;
    MsgSvcAddr* server;
    Rpc* rpc;
  public:
    Configuration(Rpc* rpc, const char* name);
    Configuration(Rpc* rpc, MsgSvcAddr* server, const char* name);
    ~Configuration();
    String* getValue(const char* section, const char* key);
    String* getString(const char* key);
    String* getString(const char* key, const char* defVal);
    int getInteger(const char* key, int defVal);
    float getFloat(const char* key, float defVal);
    Logger* getLogger(void);
    Sentuator* getSentuator(const char* name);
    EComms* getEComms(void);
    Monitor* getMonitor(void);
    int putValue(const char* section, const char* key, const char* value);
    int putString(const char* name, const char* value);
    int putString(const char* name, String* value);
    int putInteger(const char* name, int value);
    int putFloat(const char* name, float value);
    int remove(const char* name);
};

#endif
