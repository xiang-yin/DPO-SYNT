#include <unordered_map>
#include <vector>
#include <string>
#include "Typedef.h"
#include "Bimap.h"

struct D_STATE{
	STATE state_1;
	STATE state_2;
};

class D_FSM{
public:
	FSM* fsm_1;
	FSM* fsm_2;
	std::vector<std::unordered_map<EVENT, STATE>> transitions;
	std::unordered_map<STATE, std::unordered_map<STATE, D_STATE*> > dstate_map;
	Bimap<D_STATE*, STATE> states;
	int nstates; /* # of states */
	std::vector<bool> marked; /* marked states */
	
	Bimap<std::string, EVENT> events;
	int nevents; /* # of events */
	std::vector<bool> controllable; /* controllable events */
	std::vector<bool> observable; /* observable events */
	std::vector<bool> monitorable;
	std::vector<int> uu; /* uncontrollable and unobservable events */
	std::vector<int> uo; /* uncontrollable and observable events */
	D_FSM(){}

};
