#include <string>
using namespace std;
struct TaskDescriptor{
	string task;
	string next_task;
	string fallback_task;
	int timeout; //in seconds
};