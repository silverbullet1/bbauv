#include <stdio.h>
#include <string.h>
#include <dsaav/dsaav.h>
#include <enum2str.h>


class Component1 : public SentuatorService {

public:
	Component1(Rpc* rpc) : SentuatorService(rpc,"Component1")
	{		
		log->info("$Id: Component1 $");
		log->info("Init");		
		
		int param1 = 0;
		param1 = cfg->getInteger("Param1",0);
		log->info("Component1:param1 from configuration file, value = %d",param1);
	}

	~Component1(void) {	
	}

	int set(ActuatorType key, float value)
	{
		log->info("set has been called");
		return -1;
	}

	void notification(MeasurementType mt, Measurement* m)
	{
		log->info("notification is called");
	}

	void tick(void) {
		log->info("tick in Component1");
		
		//sentuator demo
		Sentuator* sServices2 = cfg->getSentuator("Service2");
		if(sServices2 == NULL)
		{
			log->warning("services2 not found");
		}
		else
		{
			//get sentuator value
			 Measurement* m = sServices2->get(MT_MEASUREMENT2,0);
			 float value = m->get(MQ_MEASUREMENT2);
			 log->info("value get from sentuator Service2 = %f",value);
			 delete m;
			 
			 //set sentuator value
			 log->info("value of 3 is set to Sentuator Service2");
			 sServices2->set(AT_ACTUATOR2,3);
		}
		delete sServices2;
		
		
		//notification demo
		Measurement* m = new Measurement(1,getTime());
      	m->put(MQ_MEASUREMENT1,4);
      	notify(MT_MEASUREMENT1,m);
      	log->info("notifying subscribers with measurement1, quantity = 4");
      	delete m;

      	//health check demo
      	Monitor* monitor = cfg->getMonitor();
      	if(monitor != NULL){
			HealthRecords* hRecord = monitor->getEverybodysHealth();
			for(int i=0;i<hRecord->count();i++)
			{
				if(hRecord->getHealth(i)!=HS_HEALTHY)
				{
					String* str = new String(hRecord->getName(i));
					str->append(" = ");
					str->append(health2str(hRecord->getHealth(i)));
					log->warning("helthMonitor report [%s]",str->chars());
					delete str;

					Severity severity = monitor->getProblemSeverity();
					if(severity==SL_ABORT||severity==SL_EMERGENCY||severity==SL_WARN)
					{
						log->warning("Sever health condition");
					}
				}
				else
				{
					log->info("health monitor reports healthy");
				}
			}
			delete hRecord;
		}
		delete monitor;
	}
};

class Component2 : public SentuatorService {

public:
	Component2(Rpc* rpc) : SentuatorService(rpc,"Component2")
	{		
		log->info("$Id: Component2 $");
		log->info("Init");		
		
		bind(MT_MEASUREMENT1);
		bind(MT_MEASUREMENT2);
		bind(AT_ACTUATOR2);
		
		int param1 = 0;
		param1 = cfg->getInteger("Param1",0);
		log->info("Component2:param1 from configuration file, value = %d",param1);
		
		String* str = cfg->getString("DEVICE","not found");
		log->info("Component2:String param from configuration file, content = %s",str->chars());
		delete str;
	}

	~Component2(void) {	
	}

	int set(ActuatorType key, float value)
	{
		if(key == AT_ACTUATOR2)
		{
			log->info("Actuator2 set with value %f",value);
			return 0;
		}
		return -1;
	}
	
	Measurement* get(MeasurementType key, float maxAge)
	{
		Measurement* m = NULL;
		if(key == MT_MEASUREMENT2)
		{
			log->info("Measurement2 requested, return with value 2");
			m = new Measurement(1,getTime());
			m->put(MQ_MEASUREMENT2,2);
		}
		return m;	
	}

	void notification(MeasurementType mt, Measurement* m)
	{
		log->info("notification is received");
		if(mt == MT_MEASUREMENT1)
		{
			float measurement1 = m->get(MQ_MEASUREMENT1);
			log->data("measurement1 received, value=%f",measurement1);
		}
	}

	void tick(void) {
		log->info("tick in Component2");
	}
	
	Health getHealth(void)
	{
		return HS_HEALTHY;
		//return HS_MALFUNCTION;
		//return HS_UNAVAILABLE;
	}
};

class container1 : public Container {

private:

	SentuatorServer* svr;
	Component1* com1;

public:
	container1(IComms* ic) : Container(ic) {
		svr = new SentuatorServer;
		rpc->bind(RPC_SENTUATOR_SET,svr);
		rpc->bind(RPC_SENTUATOR_GET,svr);
		rpc->bind(RPC_SENTUATOR_NOTIFY,svr);
		rpc->bind(RPC_HEALTH_GET,svr);
		
		com1 = new Component1(rpc);
		add(com1);
		svr->add(com1);
		
		setTickDelay(1000);
	}

	~container1(void) {
		delete com1;
	}
};

class container2 : public Container {

private:

	SentuatorServer* svr;
	Component2* com2;

public:
	container2(IComms* ic) : Container(ic) {
		svr = new SentuatorServer;
		rpc->bind(RPC_SENTUATOR_SET,svr);
		rpc->bind(RPC_SENTUATOR_GET,svr);
		rpc->bind(RPC_SENTUATOR_NOTIFY,svr);
		rpc->bind(RPC_HEALTH_GET,svr);
		
		com2 = new Component2(rpc);
		add(com2);
		svr->add(com2);
		
		setTickDelay(1500);
	}

	~container2(void) {
		delete com2;
	}
};

int main(int argc, char* argv[])
{
	bool isSim = false;
	for (int i = 1; i < argc; i++)
		if (!strcmp(argv[i],"-sim")) isSim = true; 
		
	// open appropriate communications interface
	IComms* ic = NULL;
	int port = 0xB3;
	if(isSim)
		ic = new ICommsSim(port);
	else
		ic = new ICommsEth(port);
	MsgSvcAddr* addr = ic->getAddress();
	if (addr == NULL) {
		printf("Could not open IComms interface\n");
		return 1;
	}
	String* s = addr->toString();
	printf("Running Container1 on %s\n",s->chars());
	delete s;
	delete addr;
	container1* con1 = new container1(ic);
	startContainerThread(con1);
	
	IComms* ic2 = NULL;
	int port2 = 0xB4;
	if(isSim)
		ic2 = new ICommsSim(port2);
	else
		ic2 = new ICommsEth(port2);
	MsgSvcAddr* addr2 = ic2->getAddress();
	if (addr2 == NULL) {
		printf("Could not open IComms interface\n");
		return 1;
	}
	String* s2 = addr2->toString();
	printf("Running Container2 on %s\n",s2->chars());
	delete s2;
	delete addr2;
	container2* con2 = new container2(ic2);
	con2->run();
	
	return 0;
}
