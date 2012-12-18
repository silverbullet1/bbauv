/**
 * DSAAV Library - Internal Communications Service.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.30 $
 */

#ifndef _DSAAV_ICOMMS_H_
#define _DSAAV_ICOMMS_H_

#ifndef MCU
#include <pthread.h>
#endif

/**
 * Broadcast channel identifier.
 */
enum Channel {
  CH_RPC_BCAST = 0x01      //!< RPC broadcast channel.
};

/**
 * Parameter key identifier.
 */
enum ParamKey {
  PK_MSG_ID = 0x00,          //!< Unique message identifier.
  PK_MSG_TYPE = 0x01,        //!< Message type (see MessageType).
  PK_MSG_REF = 0x02,         //!< Referred message's identifier.
  PK_MSG_SRC = 0x03,         //!< Message source address.
  PK_RPC_OP = 0x04,          //!< RPC operation (see RpcOp).
  PK_SECTION = 0x05,         //!< Configuration section name.
  PK_KEY = 0x06,             //!< Parameter key.
  PK_VALUE = 0x07,           //!< Parameter value.
  PK_COUNT = 0x08,           //!< Number of values.
  PK_AGE = 0x09,             //!< Age of the data.
  PK_MAX_AGE = 0x0A,         //!< Maximum age of a measurement.
  PK_SENSOR = 0x0B,          //!< Name of a sensor.
  PK_ACTUATOR = 0x0C,        //!< Name of an actuator.
  PK_ERROR_CODE = 0x0D,      //!< Return error code.
  PK_LOG_LEVEL = 0x10,       //!< Log level.
  PK_LOG_MODULE = 0x11,      //!< Name of module in log entry.
  PK_LOG_ENTRY = 0x12,       //!< Message to log.
  PK_ECOMMS_IFACE = 0x20,    //!< EComms interface.
  PK_ECOMMS_SRC = 0x21,      //!< EComms source address.
  PK_ECOMMS_DST = 0x22,      //!< EComms destination address.
  PK_ECOMMS_MSGCLS = 0x23,   //!< EComms message class.
  PK_ECOMMS_MSG = 0x24,      //!< EComms message (string representation in hex).
  PK_ECOMMS_MSGLEN = 0x25,   //!< EComms message length.
  PK_ECOMMS_FLAGS = 0x26,    //!< EComms flags.
  PK_ECOMMS_PRIORITY = 0x27, //!< EComms priority.
  PK_ECOMMS_LIFETIME = 0x28, //!< EComms lifetime/s.
  PK_NAME = 0x30,            //!< Component/module name.
  PK_HEALTH = 0x31,          //!< Health status.
  PK_VALIDITY = 0x32,        //!< Validity (time).
  PK_SEVERITY = 0x33,        //!< Problem severity.
  PK_FRAGMENT = 0x40,        //!< Fragment number.
  PK_DATA = 0x41,            //!< Generic data.
  PK_CHKSUM = 0x42,          //!< Checksum over data.
  PK_SERVER = 0x43,          //!< Server address.
  PK_VERSION = 0x44,         //!< Version information.
  PK_ADDRESS = 0x45,         //!< Address information (eg. SMS phone number).
  PK_USER_PARAMS = 0x50      //!< Placeholder for user-defined parameters
};

/**
 * Message type identifier.
 */
enum MessageType {
  MSG_RPC_REQ = 0x01,      //!< RPC request.
  MSG_RPC_ACK = 0x02,      //!< RPC acknowledgment.
  MSG_RPC_RSP = 0x03,      //!< RPC response.
  MSG_RPC_NTF = 0x04       //!< RPC notification (request without ack/rsp).
};

/**
 * \internal
 * Data type identifier.
 */
enum DataType {
  DT_INTEGER = 0x01,       //!< Integer (32 bit) data type identifier.
  DT_FLOAT = 0x02,         //!< Float (32 bit) data type identifier.
  DT_STRING = 0x03,        //!< String (ASCIIZ) data type identifier.
  DT_DOUBLE = 0x04         //!< Double (64 bit) data type identifier.
};

/**
 * Messaging node address.
 */
class MsgSvcAddr {
  private:
    uint64 hwAddr;
    uint16 port;
    void parse(const char* s);
  public:
    MsgSvcAddr(uint64 hwAddr, uint16 port);
    MsgSvcAddr(const char* addr);
    MsgSvcAddr(String* addr);
    MsgSvcAddr(MsgSvcAddr* addr);
    ~MsgSvcAddr(void);
    uint64 getHardwareAddr(void);
    uint16 getPort(void);
    String* toString(void);
    int operator==(MsgSvcAddr& ref);
};

/**
 * Parameter set containing key-value pairs.
 */
class ParamSet {
  protected:
    BufferPtr buf;
    int buflen, datalen;
    BufferPtr findParam(ParamKey key, DataType type, int n);
    uint16 checksum(BufferPtr buffer, int len);
  public:
    ParamSet(void);
    ParamSet(int size);
    ParamSet(ParamSet* p);
    ~ParamSet(void);
    void addInteger(ParamKey key, int value);
    void addFloat(ParamKey key, float value);
    void addDouble(ParamKey key, double value);
    void addString(ParamKey key, const char* value);
    void addString(ParamKey key, String* value);
    int getInteger(ParamKey key, int n = 0);
    float getFloat(ParamKey key, int n = 0);
    double getDouble(ParamKey key, int n = 0);
    String* getString(ParamKey key, int n = 0);
    int serialize(BufferPtr buffer);
    void unserialize(BufferPtr buffer, int inbuflen);
    String* toString(void);
};

/**
 * Communication message.
 */
class Message : public ParamSet {
  public:
    Message(void);
    Message(MessageType type);
    int getId(void);
    MessageType getType(void);
    int getRef(void);
    static int generateId(void);
};

/**
 * Internal communications manager interface.
 */
class IComms {
  public:

    /**
     * Free resources allocated for communications.
     */
    virtual ~IComms(void) {}

    /**
     * Get the address of the communication interface. The
     * address should be destroyed by the caller after use.
     *
     * \return          Address of the interface.
     */
    virtual MsgSvcAddr* getAddress() = 0;

    /**
     * Start listening for incoming messages. Incoming messages
     * are added to the incoming message queue, which can be
     * read via the getMessage method.
     */
    virtual void listen(void) = 0;

    /**
     * Stop listening for incoming messages. All incoming messages
     * are dropped.
     */
    virtual void stopListening(void) = 0;

    /**
     * Subscribe to a broadcast channel. Incoming messages on the
     * subscribed channel are added to the message queue, which can
     * be read via the getMessage method.
     *
     * \param ch        Channel identifier.
     */
    virtual void subscribe(Channel ch) = 0;

    /**
     * Unsubscribe to a broadcast channel. Incoming messages on the
     * subscribed channel are dropped.
     *
     * \param ch        Channel identifier.
     */
    virtual void unsubscribe(Channel ch) = 0;

    /**
     * Send message to a known destination. This method provides an
     * unreliable message delivery. Calling method may destroy
     * \p dst and \p msg.
     *
     * \param dst       Destination address.
     * \param msg       Message to be transmitted.
     */
    virtual void sendMessage(MsgSvcAddr* dst, Message* msg) = 0;

    /**
     * Send message to all listeners on a broadcast channel. This
     * method provides an unreliable message delivery. Calling method
     * may destroy \p msg.
     *
     * \param ch        Broadcast channel identifier.
     * \param msg       Message to be transmitted.
     */
    virtual void broadcastMessage(Channel ch, Message* msg) = 0;

    /**
     * Get a pending message from the input message queue. This method
     * does not block if no messages are available. Calling method should
     * free the message after use.
     *
     * \return          Pending message, NULL if none available.
     */
    virtual Message* getMessage(void) = 0;

    /**
     * Wait for a message or until timeout.
     *
     * \param timeout   Timeout in milliseconds.
     */
    virtual void waitForMessage(int timeout) = 0;
};

#if defined(LINUX) || defined(OSX)
/**
 * Internal communications driver for Ethernet.
 */
class ICommsEth : public IComms {
  private:
    int rxSock, txSock;
    byte ethAddr[6];
    char ifname[16];
    int port;
    int ifindex;
    List* subs;
    List* msgQueue;
    int listening;
    pthread_mutex_t slock, qlock;
    pthread_cond_t msgwait;
    void sendEthPacket(uint16 hdr, uint64 dstEthAddr, Message* msg);
    static void* listener(void* ref);
  public:
    ICommsEth(int port, const char* ifname = NULL);
    ~ICommsEth(void);
    MsgSvcAddr* getAddress();
    void listen(void);
    void stopListening(void);
    void subscribe(Channel ch);
    void unsubscribe(Channel ch);
    void sendMessage(MsgSvcAddr* dst, Message* msg);
    void broadcastMessage(Channel ch, Message* msg);
    Message* getMessage(void);
    void waitForMessage(int timeout);
};
#endif

#ifdef MCU
/**
 * Internal communications driver for Ethernet.
 */
class ICommsEth : public IComms {
  private:
    uint64 addr;
    int port;
    int listening;
    List* subs;
    Message* pendingMsg;
    void sendEthPacket(uint16 hdr, uint64 dstEthAddr, Message* msg);
    Message* getEthPacket(void);
  public:
    ICommsEth(int port);
    ~ICommsEth(void);
    MsgSvcAddr* getAddress();
    void listen(void);
    void stopListening(void);
    void subscribe(Channel ch);
    void unsubscribe(Channel ch);
    void sendMessage(MsgSvcAddr* dst, Message* msg);
    void broadcastMessage(Channel ch, Message* msg);
    Message* getMessage(void);
    void waitForMessage(int timeout);
};
#endif

#if defined(LINUX) || defined(OSX)
/**
 * Internal communications driver for single process simulation.
 */
class ICommsSim : public IComms {
  private:
    static float ploss;
    int myAddr;
    List* subs;
    int listening;
    int qid;
    Message* lastmsg;
    void flushQueue(int qid);
    void cleanUpQueues(void);
  public:
    ICommsSim(int addr);
    ~ICommsSim(void);
    MsgSvcAddr* getAddress();
    void listen(void);
    void stopListening(void);
    void subscribe(Channel ch);
    void unsubscribe(Channel ch);
    void sendMessage(MsgSvcAddr* dst, Message* msg);
    void broadcastMessage(Channel ch, Message* msg);
    Message* getMessage(void);
    void waitForMessage(int timeout);
    static void setPacketLoss(float p);
};
#endif

#endif
