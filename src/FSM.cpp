#include <queue>
#include <cstring>
#include <cstdlib>
#include "../include/FSM.h"
using namespace std;

FSM::FSM(const int nstates_, const int nevents_)
	: nstates(nstates_), nevents(nevents_) {
	resize();
}

FSM::FSM(const string& file_in, Mode mode_ /* = BSCOPNBMAX */) : mode(mode_) {
	ifstream input(file_in.c_str());
	if (!input.is_open()) {
		cerr << "Error: file \'" << file_in << "\' could not be read\n";
		cin.get();
		exit(1);
	}
	if (file_in.find(".fsm") != string::npos) {
		input >> nstates;
		nevents = 0;
		resize();
		read_fsm_input(input);
	}
	else {
		input >> nstates >> nevents;
		resize();
		read_txt_input(input);
	}
}

void FSM::resize() {
	transitions.resize(nstates);
	states.reserve(nstates);
	marked.resize(nstates, false);
	events.reserve(nevents);
	controllable.resize(nevents);
	observable.resize(nevents);
	monitorable.resize(nevents);
}

void FSM::read_txt_input(ifstream& input) {
	/* Create state bimap */
	string state;
	char m; /* marked */
	for(STATE s = 0; s < nstates; ++s) {
		input >> state >> m;
		states.insert(state, s);
		if (m == 'm') marked[s] = true;
	}
	/* Create event bimap */
	string event;
	char c, o; /* controllable and observable */
	for (EVENT e = 0; e < nevents; ++e) {
		input >> event >> c >> o;
		events.insert(event, e);
		if (c == 'c') {
			if (mode == MPO) monitorable[e] = true;
			else {
				controllable[e] = true;
				if (o == 'o') observable[e] = true;
			}
		}
		else {
			if (o == 'o') {
				observable[e] = true;
				uo.push_back(e);
			}
			else if (o == 'm') monitorable[e] = true;
			else uu.push_back(e);
		}
	}
	/* Read in transition function */
	STATE s = -2;
	while (input.peek() == '\n') { 
		input.ignore();
		++s;
	}
	string new_state;
	while (input >> event && input >> new_state) {
		transitions[s][events.get_value(event)] = states.get_value(new_state);
		/* May be run more than once if some state(s) has out-degree = 0 */
		while (input.peek() == '\n') { 
			input.ignore();
			++s;
		}
	}
}

void FSM::read_fsm_input(ifstream& input) {
	/* Map unique strings to basic types */
	int state_index = 0, event_index = 0;
	string state;
	bool is_marked;
	int num_transitions;
	/* For each state */
	while (input >> state >> is_marked >> num_transitions) {
		/* Add parent state info */
		if (!states.find_value(state))
			states.insert(state, state_index++);
		STATE parent = states[state];
		marked[parent] = is_marked;
		/* For each corresponding state transition */
		string event, child_state, c, o;
		while (num_transitions-- && input >> event >> child_state >> c >> o) {
			/* Add child state info */
			if (!states.find_value(child_state))
				states.insert(child_state, state_index++);
			/* Add event info */
			if (!events.find_value(event)) {
				events.insert(event, event_index);
				bool is_controllable = c == "c";
				bool is_observable = o == "o";
				if (mode == MPO) {
					monitorable.push_back(is_controllable);
					controllable.push_back(false);
				}
				else {
					controllable.push_back(is_controllable);
					monitorable.push_back(false);
				}
				observable.push_back(is_observable);
				if (!is_controllable && !is_observable)
					uu.push_back(event_index);
				if (!is_controllable && is_observable)
					uo.push_back(event_index);
				++event_index;
			}
			/* Add transition */
			STATE child = states[child_state];
			EVENT link = events[event];
			transitions[parent][link] = child;
 		}
	}
	nevents = events.size();
}

STATE FSM::find_inaccessible(vector<int>& current_access) {
	for (STATE s = 0; s < current_access.size(); ++s)
		if (!current_access[s]) return s;
	return -1;
}

/* A FSM will produce an empty NBAIC if there are no states, or if there
are no accessible marked states */
bool FSM::is_invalid(Mode mode) {
	return states.empty()
		|| (!exists_accessible_marked_state() && mode == BSCOPNBMAX);
}

bool FSM::exists_accessible_marked_state() {
	vector<bool> visited(nstates, false);
	queue<STATE> BFS; /* Breadth First Search */
	BFS.push(0); /* Start state is always zero */
	while(!BFS.empty()) {
		STATE current = BFS.front();
		BFS.pop();

		if (visited[current]) continue;
		visited[current] = true;
		/* Found an accessible, marked state */
		if (marked[current]) return true;
		for (auto& pair : transitions[current])
			BFS.push(pair.second);
	}
	/* No accessible state is marked */
	return false;
}


void FSM::print_txt(ostream& os) {
	os << nstates << ' ' << nevents << "\n\n";
	/* Print list of states in format <state_name marked> */

	for (STATE s = 0; s < nstates; ++s)
		os << states.get_key(s) << ' ' << (marked[s] ? 'm' : 'u') << '\n';
	os << '\n';
	/* Print events in format <event_name controllability observability> */
	for (EVENT e = 0; e < nevents; ++e)
		os << events.get_key(e)
		   << ' ' << (controllable[e] ? 'c' : 'u')
		   << ' ' << (observable[e] ? 'o' : 'u') << '\n';
	os << '\n';
	/* Print transition function for each state--one state on each line--
	in same order as the printed state list */

	for (STATE s = 0; s < nstates; ++s) {
		bool first_iter = true;
		for(auto& transition : transitions[s]) {
			os << (first_iter ? "" : " ") << events.get_key(transition.first)
			   << ' ' << states.get_key(transition.second);
			first_iter = false;
		}
		os << '\n';
	}
}

void FSM::print_fsm(const char* const filename) {
	ofstream file_out(filename);
	print_fsm(file_out);
	file_out.close();
}

void FSM::print_fsm(ostream& os) {
	os << nstates << "\r\n\r\n";
	for (STATE s = 0; s < nstates; ++ s) {
		os << states.get_key(s) << "	" << marked[s]
		   << "	" << transitions[s].size() << "\r\n";
		for (auto& pair : transitions[s]) {
			os << events.get_key(pair.first) << "	"
			   << states.get_key(pair.second) << "	"
			   << (controllable[pair.first] ? "c" : "uc") << "	"
			   << (observable[pair.first] ? "o" : "uo") << "\r\n";
		}
		os << "\r\n";
	}
}

/* Reduce the A_UxG to a more compact and readable form */
void FSM::reduce(ostream& os) {
	/* Scale the FSM state to be a number between 0 and nstates */
	Bimap<string, int> state_scaler(nstates);
	/* Table of FSM states and the A_UxG states that contain them */
	unordered_map<int, unordered_map<string, int>> state_index;
	state_index.reserve(nstates);
	/* Keep track of how many times we see a FSM state in
	different A_UxG states */
	vector<int> state_count(nstates);
	int scaler_count = 0;
	os << nstates << "\r\n\r\n";
	for (STATE s = 0; s < nstates; ++s) {
		string parent_key = states.get_key(s);
		STATE parent = find_state(parent_key, state_scaler, scaler_count);
		string parent_primes = get_primes(state_index, state_count,
										  parent, parent_key);
		os << state_scaler.get_key(parent) << parent_primes << "	" << marked[s]
		   << "	" << transitions[s].size() << "\r\n";
		for (auto& pair : transitions[s]) {
			string child_key = states.get_key(pair.second);
			STATE child = find_state(child_key, state_scaler, scaler_count);
			string child_primes = get_primes(state_index, state_count,
											 child, child_key);
			os << events.get_key(pair.first) << "	"
			   << state_scaler.get_key(child) << child_primes << "	"
			   << (controllable[pair.first] ? "c" : "uc") << "	"
			   << (observable[pair.first] ? "o" : "uo") << "\r\n";
		}
		os << "\r\n";
	}
}

/* Extract the FSM state from the A_UxG state and map it to a unique int
between 0 and nstates */
STATE FSM::find_state(string& A_UxG_State, Bimap<string, int>& state_scaler,
					  int& scaler_count) {
	auto last_comma = A_UxG_State.find_last_of(',');
	auto prev_comma = last_comma - 1;
	int num_paren = 0, num_braces = 0, num_bracket = 0;
	/* Find the next comma that is not contained
	within some enclosing punctuation */
	for (; prev_comma > 0; --prev_comma) {
		char current = A_UxG_State[prev_comma];
		if (current == ',' && num_paren == 0 &&
			num_bracket == 0 && num_braces == 0) break;
		else if (current == '(') --num_paren;
		else if (current == '{') --num_braces;
		else if (current == '[') --num_bracket;
		else if (current == ')') ++num_paren;
		else if (current == '}') ++num_braces;
		else if (current == ']') ++num_bracket;
	}
	string s = A_UxG_State.substr(prev_comma + 1, last_comma - prev_comma - 1);
	/* Have not seen this FSM state before--create unique mapping */
	if (!state_scaler.find_value(s))
		state_scaler.insert(s, scaler_count++);
	return state_scaler[s];
}

/* Return string of primes that differentiates the FSM state from any other
A_UxG states with the same FSM state */
string FSM::get_primes(unordered_map<int, unordered_map<string, int>>& state_index,
				  	   vector<int>& state_count, STATE s, string& state_key) {
	int result = 0;
	/* Have not seen this A_UxG state before--update # of primes */
	if (state_index[s].find(state_key) == state_index[s].end()) {
		result = state_count[s]++;
		state_index[s][state_key] = result;
	}
	/* A_UxG state has been seen before--use precomputed value */
	else result = state_index[s][state_key];
	return string(result, '\'');
}
