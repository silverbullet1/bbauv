/**
 * DSAAV Library - Remote Procedure Call (RPC) Service.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.23 $
 */

#ifndef _DSAAV_RPC_H_
#define _DSAAV_RPC_H_

//#define RPC_TRACE
//#define RPC_TRACE_STDOUT

#ifdef RPC_TRACE
#include <stdio.h>
#endif
#ifdef EXPERIMENTAL
#ifndef MCU
#include <pthread.h>
#endif
#endif

static const int MAX_NESTING = 10;        //!< Maximum RPC call nest level
static const int RPC_ACK_TIMEOUT = 100;   //!< RPC ack timeout (ms).
static const int RPC_RSP_TIMEOUT = 500;   //!< RPC response timeout (ms).
static const int MAX_RTM_COUNT = 5;       //!< Retransmission counter.
static const int CACHE_TIMEOUT = 600;     //!< Cache clearing timeout (ms).
#ifdef MCU
static const int CACHE_SIZE = 32;         //!< Maximum number of cache entries.
#else
static const int CACHE_SIZE = 8192;       //!< Maximum number of cache entries.
#endif

/**
 * RPC operation identifier.
 */
enum RpcOp {
  RPC_CFG_GET = 0x00,          //!< Get configuration entry.
  RPC_LOG = 0x01,              //!< Log message.
  RPC_SENTUATOR_GET = 0x02,    //!< Get sensor measurement.
  RPC_SENTUATOR_SET = 0x03,    //!< Set value of actuator.
  RPC_SENTUATOR_NOTIFY = 0x04, //!< Notify listener of measurement.
  RPC_CCOMMS_GET = 0x05,       //!< C3 communications getter method.
  RPC_CCOMMS_SET = 0x06,       //!< C3 communications setter method.
  RPC_ECOMMS_SEND = 0x07,      //!< EComms send primitive.
  RPC_ECOMMS_RECEIVE = 0x08,   //!< EComms receive primitive.
  RPC_HEALTH_GET = 0x09,       //!< Get health.
  RPC_HEALTH_GETALL = 0x0A,    //!< Get everybody's health.
  RPC_HEALTH_SET = 0x0B,       //!< Set health.
  RPC_SEVERITY_GET = 0x0C,     //!< Get problem severity.
  RPC_UPDATE_CHECK = 0x0D,     //!< Check for availability of software update.
  RPC_UPDATE_GET = 0x0E,       //!< Get software update fragment.
  RPC_SHELL_EXEC = 0x0F,       //!< Execute shell command.
  RPC_CFG_PUT = 0x10,          //!< Put configuration persistent store entry.
  RPC_MESSAGE = 0x11,          //!< Deliver a message (used for SMS send/receive).
  RPC_LASTOP = 0x12            //!< \internal Dummy operation to get count.
};

/**
 * Base class for a RPC service provider.
 */
class RpcService {
  protected:
    RpcService(void) { }
  public:
    virtual ~RpcService(void) { }
    virtual ParamSet* service(ParamSet* params) = 0;
};

/**
 * RPC server and client.
 */
class Rpc {
  private:
    MsgSvcAddr* me;
    RpcService* opHandler[RPC_LASTOP];
    IComms* comms;
    int refs[MAX_NESTING];
    Message* retvals[MAX_NESTING];
    int acks[MAX_NESTING];
    int counter;
    List* statusList;
    Message* msg_ack;
#ifdef EXPERIMENTAL
#ifndef MCU
    pthread_mutex_t mutex;
#endif
#endif
#ifdef RPC_TRACE
    FILE* traceOut;
    void trace(const char* fmt, ...);
    void tracePS(const char* msg, ParamSet* ps);
#endif
    void processMessage(Message* msg);
  public:
    Rpc(IComms* ic);
    ~Rpc(void);
    void bind(RpcOp op, RpcService* svc);
    void process(void);
    void process(int timeout);
    ParamSet* call(RpcOp op, ParamSet* params);
    ParamSet* call(MsgSvcAddr* host, RpcOp op, ParamSet* params);
    ParamSet* call(MsgSvcAddr* host, RpcOp op, ParamSet* params, int ackto, int rspto, int maxrtm);
#ifdef RPC_TRACE
    void setTraceOutput(FILE* fp);
#endif
};

#endif
