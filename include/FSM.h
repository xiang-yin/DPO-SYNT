#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Typedef.h"
#include "Bimap.h"

/* Finite State Machine */
class FSM {
public:
	FSM(const int nstates_, const int nevents_);
	FSM(const std::string& file_in, Mode mode_ = BSCOPNBMAX);
	void print_txt(std::ostream& os);
	void print_fsm(const char* const filename);
	void print_fsm(std::ostream& os);
	void reduce(std::ostream& os);
	STATE find_inaccessible(std::vector<int>& current_access);
	bool is_invalid(Mode mode);

	std::vector<std::unordered_map<EVENT, STATE>> transitions;
	
	Bimap<std::string, STATE> states;
	int nstates; /* # of states */
	std::vector<bool> marked; /* marked states */
	
	Bimap<std::string, EVENT> events;
	int nevents; /* # of events */
	std::vector<bool> controllable; /* controllable events */
	std::vector<bool> observable; /* observable events */
	std::vector<bool> monitorable;
	std::vector<int> uu; /* uncontrollable and unobservable events */
	std::vector<int> uo; /* uncontrollable and observable events */

private:
	Mode mode;
	void resize();
	void read_txt_input(std::ifstream& input);
	void read_fsm_input(std::ifstream& input);
	STATE find_state(std::string& A_UxG_State,
					 Bimap<std::string, int>& state_scaler, int& scaler_count);
	std::string get_primes(std::unordered_map<int, std::unordered_map<std::string, int>>& state_index,
					  	   std::vector<int>& state_count, STATE parent,
					  	   std::string& state_key);
	bool exists_accessible_marked_state();
};

#endif
