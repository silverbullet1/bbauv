#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <dsaav/dsaav.h>
#include <dsaav/coresvr.h>
#include <enum2str.h>

static char myaddr[64];
static char* logfilename = NULL;

class CoreContainer : public Container {
private:
  ConfigurationServer* cfgSvr;
  LoggingServer* logSvr;
  MonitorServer* monSvr;

public:
  CoreContainer(IComms* ic, const char* cfgfile = NULL);
  ~CoreContainer(void);
  void tick(void);

};

CoreContainer::CoreContainer(IComms* ic, const char* cfgfile) : Container(ic)
{

    if (cfgfile != NULL) cfgSvr = new ConfigurationServer(cfgfile,myaddr);
    else cfgSvr = new ConfigurationServer(myaddr);
    rpc->bind(RPC_CFG_GET,cfgSvr);
    rpc->bind(RPC_CFG_PUT,cfgSvr);
    printf("Configuration server ready\n");

    if (logfilename == NULL) logSvr = new LoggingServer; 
    else logSvr = new LoggingServer(logfilename);
    rpc->bind(RPC_LOG,logSvr);
    printf("Logging server ready\n");
    
    monSvr = new MonitorServer(rpc,ic->getAddress());
    rpc->bind(RPC_HEALTH_GET,monSvr);
    rpc->bind(RPC_HEALTH_GETALL,monSvr);
    rpc->bind(RPC_HEALTH_SET,monSvr);
    rpc->bind(RPC_SEVERITY_GET,monSvr);
    printf("Monitor server ready\n");

    setTickDelay(10);
}

CoreContainer::~CoreContainer(void)
{
  if (cfgSvr) delete cfgSvr;
  if (logSvr) delete logSvr;
  if (monSvr) delete monSvr;
}

void CoreContainer::tick(void)
{

  if (cfgSvr != NULL) cfgSvr->tick();
  if (monSvr != NULL) monSvr->tick();
  
}

int main(int argc, char* argv[])
{
	bool isSim = false;
	for (int i = 1; i < argc; i++)
		if (!strcmp(argv[i],"-sim")) isSim = true;
		
  IComms* ic = NULL;
  int port = 0xF1;

  if (isSim) 
    ic = new ICommsSim(port);
  else
  	ic = new ICommsEth(port);
  MsgSvcAddr* addr = ic->getAddress();
  if (addr == NULL) {
    printf("Could not open IComms interface\n");
    return 1;
  }
  String* s = addr->toString();
  printf("Running on %s\n",s->chars());
  strcpy(myaddr,s->chars());
  char* p = strchr(myaddr,':');
  if (p != NULL) *p = 0;
  delete s;
  delete addr;
  CoreContainer* cc = new CoreContainer(ic,"dsaav-sim.cfg");
  cc->run();

  return 0;
}
