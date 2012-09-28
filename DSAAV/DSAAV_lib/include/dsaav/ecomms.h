/**
 * DSAAV Library - External Communications Interface.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.5 $
 */

#ifndef _DSAAV_ECOMMS_H_
#define _DSAAV_ECOMMS_H_

// Ecomms data types
typedef int ECommsAddr;                     //!< EComms address type.

// Ecomms general constants
const int CCL_MSG_LEN = 32;                 //!< CCL message length.
const ECommsAddr ECA_BCAST = 0xff;          //!< EComms broadcast address.

/**
 * Ecomms message priorities
 */
enum ECommsPriority {
  ECP_PRIORITY1,                            //!< Mostly Time Critical
  ECP_PRIORITY2                             //!< Non-time critical
};

/**
 * Ecomms communications interfaces.
 */
enum ECommsIface {
  ECIF_AUTO,                                //!< Auto select best interface.
  ECIF_ACOMMS,                              //!< Acoustic communications interface.
  ECIF_WIFI,                                //!< 802.11 wireless networking interface.
  ECIF_ETH,                                 //!< Wired ethernet interface (tethered mode).
  ECIF_GSM                                  //!< GSM modem interface.
};

/**
 * Ecomms message classes.
 */
enum ECommsMsgClass {
  ECMC_CCL = 0x01,                                  //!< CCL message class.
  ECMC_CCLSHORT = 0x02
  };

/**
 * External communications interface.
 *
 * This class provides a RPC interface to the external communications
 * service. This interface is usually retrieved through the configuration
 * service using the Configuration::getEcomms() method.
 *
 * EComms currently supports only CCL messages. Additional message classes
 * may be added later.
 *
 * All incoming messages are delivered using the RPC_ECOMMS_RECEIVE call
 * to the listener. Only one listener may be registered for each class
 * of messages. The listener must implement a RpcService, bind it and listen
 * for RPC requests. The listener is defined in the configuration file. E.g.
 *
 * \code
 * [EComms]
 * Server = 214365badcfe:0
 * Listener:CCL = 123456abcdef:0
 * \endcode
 */
class EComms {
  private:
    MsgSvcAddr* server;
    Rpc* rpc;
  public:
    EComms(Rpc* rpc, MsgSvcAddr* server);
    ~EComms(void);
    int sendCCLMessage(ECommsIface iface, ECommsAddr dst, BufferPtr msg);
    int sendCCLMessage(ECommsIface iface, ECommsAddr dst, BufferPtr msg, 
      ECommsPriority priority, int lifetime);
};

#endif
