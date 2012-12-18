/**
 * DSAAV Library - Component & Container Framework.
 *
 * (c) 2007 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.8 $
 */

#ifndef _DSAAV_COMPONENT_H_
#define _DSAAV_COMPONENT_H_

static const int DEFAULT_TICK_DELAY = 1000;

/**
 * Base class for a STARFISH component.
 *
 * A component is a software module with an independent
 * existance. A component usually extends the Component
 * class. It communicates to other components via
 * RPC. A component has a name and corresponding entries
 * in the configuration files and logs.
 *
 * Passive components may react to events, usually from
 * incoming RPC requests. Such components implement the
 * RpcService interface and register their presence with
 * the RPC service in the constructor.
 *
 * Active components require processing time slice on a
 * regular basis. They override the tick() method. This
 * method is called automatically by the container in
 * which the component is registered. The frequency of
 * call depends on the container settings.
 */
class Component {
  protected:
    Rpc* rpc;               //!< RPC service provider.
    Configuration* cfg;     //!< Configuration service provider.
    Logger* log;            //!< Logging service provider.
    Component(Rpc* rpc, const char* name);
  public:
    virtual ~Component(void);
    virtual void tick(void);
    virtual void shutDown(void);
};

/**
 * Base class for a STARFISH container.
 *
 * A container provides a deployable software unit as a
 * single thread. Multi-tasking OS such as Linux can
 * support multiple containers in a process (multi-threaded)
 * or in seperate processes. Other devices such as
 * microcontrollers with no multi-tasking support can
 * run a single container.
 *
 * A container may contain multiple components. The default
 * tick() method calls the Component::tick() method of each
 * component. The run() method periodically calls the tick()
 * method. The period is controlled by the setTickDelay()
 * method.
 *
 * Once a container is constructed, its run() method must
 * be called. This method is not expected to return. In
 * single-threaded processes, this method is called directly
 * after initialization. In multi-threaded processes, this
 * method is called indirectly using the startContainerThread()
 * helper function.
 */
class Container {
  protected:
    int tickDelay;        //!< Maximum delay between tick() calls.
    IComms* ic;           //!< Communications interface.
    Rpc* rpc;             //!< RPC service provider.
    List* components;     //!< List of all components managed by the container.
    Container(IComms* ic);
    Container(Rpc* rpc);
  public:
    virtual ~Container(void);
    void setTickDelay(int t);
    void add(Component* c);
    virtual void tick(void);
    virtual void run(void);
    virtual void shutDown(void);
};

#ifndef MCU
#include <pthread.h>
pthread_t startContainerThread(Container* c);
#endif

#endif
