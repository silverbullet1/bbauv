/**
 * DSAAV Library - Sensor/Actuator Server Classes.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.6 $
 */

#ifndef _DSAAV_SASVR_H_
#define _DSAAV_SASVR_H_

/**
 * \internal
 * Notification entry for sentuator service framework.
 */
struct SentuatorNotification {
  MeasurementType mtype;
  MsgSvcAddr* addr;
};

/**
 * Sensor or actuator service framework.
 */
class SentuatorService : public Component {
  protected:
    List* atypes;
    List* mtypes;
    int ntfCount;
    SentuatorNotification* ntf;
    void bind(ActuatorType type);
    void bind(MeasurementType type);
    void notify(MeasurementType type, Measurement* m);
    SentuatorService(Rpc* rpc, const char* name);
  public:
    virtual ~SentuatorService(void);
    List* getActuatorTypes(void);
    List* getMeasurementTypes(void);
    virtual int set(ActuatorType type, float value);
    virtual int set(ActuatorType type, int count, float* values);
    virtual Measurement* get(MeasurementType type, float maxAge);
    virtual void notification(MeasurementType type, Measurement* m);
    virtual Health getHealth(MeasurementType type);
    virtual Health getHealth(ActuatorType type);
    virtual Health getHealth(void);
};

/**
 * Sensor or actuator server framework.
 */
class SentuatorServer : public RpcService {
  private:
    SentuatorService* asvc[AT_LAST];
    SentuatorService* msvc[MT_LAST];
    SentuatorService** svcs;
    int svcsCnt;
  public:
    SentuatorServer(void);
    ~SentuatorServer(void);
    void add(SentuatorService* svc);
    ParamSet* service(ParamSet* params);
};

#endif
