#include <algorithm>
#include <sstream>
#include <queue>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include "../../include/FSM.h"
using namespace std;

const int MAX_STATES = 40;
const int MAX_EVENTS = 25;
const int NUM_ITERATIONS = 1;
const int MARKED_MIN = 15;
const int MARKED_MAX = 20;
const int OBSERVABLE_MIN = 50;
const int OBSERVABLE_MAX = 60;
const int CONTROLLABLE_MIN = 50;
const int CONTROLLABLE_MAX = 60;
const double MIN_DENSITY_RATIO = 0.2;
const double MAX_DENSITY_RATIO = 0.3;

const char* const DATA_FILE = "./test/scalability_test/size_data.txt";
const char* const NBAIC_FILE = "./test/scalability_test/results/NBAIC_size_results.csv";
const char* const EBTS_FILE = "./test/scalability_test/results/EBTS_size_results.csv";
const char* const RUNTIME_FILE = "./test/scalability_test/results/runtime_results.csv";

const char* const EXECUTABLE = "./bin/DES_Supervisor_test -w -f ";
const char* const EXTENSION = ".txt";
const char* const REDIRECTION = " > ./test/scalability_test/program_output.txt 2>&1";
const char* const REMOVE_TESTS = "rm -f ./test/scalability_test/rand_test_*";
const char* const REMOVE_DATA_FILE = "rm -f ./test/scalability_test/size_data.txt";

struct Datum {
	Datum() : NBAIC_size(0), EBTS_size(0), runtime(0) {}
	Datum(int NBAIC_size_, int EBTS_size_, double runtime_)
		: NBAIC_size(NBAIC_size_), EBTS_size(EBTS_size_), runtime(runtime_) {}
	void update(int NBAIC_up, int EBTS_up, double runtime_up);
	void average();
	int NBAIC_size;
	int EBTS_size;
	double runtime;
};

void create_test_file(const string&, int states, int events);
void time_test(const string& filename, int states,
			   int events, vector<vector<Datum>>& data_table,
			   int& exit_status);
void output_test_results(vector<vector<Datum>>& data_table);
FSM create_initial_FSM(int states, int events);
void make_accessible(FSM& fsm);
void do_BFS(queue<int>& BFS, int& current,
			vector<int>& current_access, FSM& fsm);
bool transition_is_invalid(FSM& fsm, vector<int>& current_access,
						   int state_out, int event_out);
void print_time_table(vector<vector<Datum>>& data_table);
int get_out_degree(int events);
bool flip_rigged_coin(int min_chance, int max_chance);
void record_data(vector<vector<Datum>>& data_table, double runtime,
				 int states, int events);
bool no_unfolds();

int main() {
	srand(time(NULL));
	vector<vector<Datum>> data_table(MAX_STATES);
	for (auto& vec : data_table) vec.resize(MAX_EVENTS);
	int count = 0;
	int exit_status = 0;
	while (count++ < NUM_ITERATIONS) {
	// while (exit_status == 0) {
	// while (no_unfolds()) {
		for (int i = MAX_STATES; i <= MAX_STATES; ++i) {
			for (int j = MAX_EVENTS; j <= MAX_EVENTS; ++j) {
				cout << i << ' ' << j << ' ' << count++ << '\n';
				const string filename("./test/scalability_test/rand_test_"
									  + to_string(i) + '_'
									  + to_string(j) + '_'
									  + to_string(count) + EXTENSION);
				create_test_file(filename, i, j);
				time_test(filename, i, j, data_table, exit_status);
				const string remove_file("rm " + filename);
				// if (no_unfolds()) system(remove_file.c_str());
			}
			// print_time_table(data_table);
		}
	}
	for (auto& vec : data_table)
		for (Datum& datum : vec)
			datum.average();
	output_test_results(data_table);
	// system(REMOVE_TESTS);
	system(REMOVE_DATA_FILE);
	return 0;
}

/* Create random input file with i states and MAX_EVENTS events */
void create_test_file(const string& filename, int states, int events) {
	FSM fsm = create_initial_FSM(states, events);
	make_accessible(fsm);
	ofstream file_out(filename);
	if (EXTENSION == ".txt") fsm.print_txt(file_out);
	else if (EXTENSION == ".fsm") fsm.print_fsm(file_out);
	file_out.close();
}

FSM create_initial_FSM(int states, int events) {
	FSM fsm(states, events);
	/* Write the state information */
	for (int k = 0; k < states; ++k) {
		fsm.states.insert(to_string(k), k);
		fsm.marked[k] = flip_rigged_coin(MARKED_MIN, MARKED_MAX);
	}
	/* Write the event information */
	for (int k = 0; k < events; ++k) {
		fsm.events.insert(to_string(k), k);
		fsm.observable[k] = flip_rigged_coin(OBSERVABLE_MIN, OBSERVABLE_MAX);
		fsm.controllable[k] = flip_rigged_coin(CONTROLLABLE_MIN, CONTROLLABLE_MAX);
	}
	/* Write the transition information */
	for (int k = 0; k < states; ++k) {
		int out_degree = get_out_degree(events);
		/* No event should be used more than once per state */
		vector<bool> determinism_checker(events, false);
		for (; out_degree > 0; --out_degree) {
			int event_out = rand() % events;
			/* Find an unused event */
			while (determinism_checker[event_out])
				event_out = (event_out + 1) % events;
			determinism_checker[event_out] = true;
			fsm.transitions[k][event_out] = rand() % states;
		}
	}
	return fsm;
}

void make_accessible(FSM& fsm) {
	vector<int> current_access(fsm.nstates);
	int current = 0; //Initial state
	queue<int> BFS; //Breadth First Search
	BFS.push(current); 
	do_BFS(BFS, current, current_access, fsm);
	/* While there exists an inaccessible state */
	int inaccessible_state = fsm.find_inaccessible(current_access);
	while(inaccessible_state != -1) {
		int event_out = rand() % fsm.nevents;
		int state_out = rand() % fsm.nstates;
		while (transition_is_invalid(fsm, current_access, state_out, event_out)) {
			state_out = rand() % fsm.nstates;
			event_out = rand() % fsm.nevents;
		}
		/* Add transition from accessible state to inaccessible state */
		fsm.transitions[state_out][event_out] = inaccessible_state;
		BFS.push(inaccessible_state);
		do_BFS(BFS, inaccessible_state, current_access, fsm);
		inaccessible_state = fsm.find_inaccessible(current_access);
	}
}

void do_BFS(queue<int>& BFS, int& current,
			vector<int>& current_access, FSM& fsm) {
	while(!BFS.empty()) {
		current = BFS.front();
		BFS.pop();
		/* Don't visit the same state more than once,
		but keep track of number of ways to visit state */
		if (current_access[current]++) continue;
		/* Push children */
		for (auto& pair : fsm.transitions[current])
			if (current != pair.second) BFS.push(pair.second);
	}
}

bool transition_is_invalid(FSM& fsm, vector<int>& current_access,
						   int state_out, int event_out) {
	/* Does not connect an accessible state to an inaccessible state */
	if (!current_access[state_out]) return true;
	/* Event is already defined for state */
	else if (fsm.transitions[state_out].find(event_out)
		  != fsm.transitions[state_out].end()) {
		/* Event transitions to a state that would be otherwise inaccessible */
		if (current_access[fsm.transitions[state_out][event_out]] == 1
			&& fsm.transitions[state_out][event_out] != state_out)
			return true;
	}
	return false;
}

void time_test(const string& filename, int states,
			   int events, vector<vector<Datum>>& data_table,
			   int& exit_status) {
	using namespace chrono;
	const string command(EXECUTABLE + filename);
	/* Start the timer */
	high_resolution_clock::time_point tp_start = high_resolution_clock::now();
	/* Run the program on random input */
	exit_status = system(command.c_str());
	/* End the timer */
	high_resolution_clock::time_point tp_end = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(tp_end - tp_start);
	record_data(data_table, time_span.count(), states, events);
}

void output_test_results(vector<vector<Datum>>& data_table) {
	ofstream NBAIC_out(NBAIC_FILE);
	ofstream EBTS_out(EBTS_FILE);
	ofstream runtime_out(RUNTIME_FILE);
	for (int i = 0; i < MAX_STATES; ++i) {
		for (int j = 0; j < MAX_EVENTS; ++j) {
			Datum& datum = data_table[i][j];
			NBAIC_out << datum.NBAIC_size << (j == MAX_EVENTS - 1 ? "\n" : ",");
			EBTS_out << datum.EBTS_size << (j == MAX_EVENTS - 1 ? "\n" : ",");
			runtime_out << datum.runtime << (j == MAX_EVENTS - 1 ? "\n" : ",");
			cout << i + 1 << " states and " << j + 1 << " events in "
				 << datum.runtime << " seconds.\n";
		}
	}
	NBAIC_out.close();
	EBTS_out.close();
	runtime_out.close();
}

void print_time_table(vector<vector<Datum>>& data_table) {
	for (auto& vec : data_table) {
		for (Datum& datum : vec)
			cout << datum.runtime << ' ';
		cout << '\n';
	}
}

/* Returns coin flip of coin with geometrically distributed chance of success
   min_chance and max_chance must be between 0 and 100 and min_chance < max_chance */
bool flip_rigged_coin(int min_chance, int max_chance) {
	int difference = max_chance - min_chance;
	int actual_chance = rand() % difference + min_chance;
	return rand() % 100 < actual_chance;
}

/* Returns the number of transitions that exit a state */
int get_out_degree(int events) {
	int min_degree = max(MIN_DENSITY_RATIO * events, 1.0);
	int max_degree = max(MAX_DENSITY_RATIO * events, 1.0);
	int difference = max_degree - min_degree;
	return difference > 0 ? rand() % difference + min_degree : 1;
}

void record_data(vector<vector<Datum>>& data_table, double runtime,
				 int states, int events) {
	ifstream size_data(DATA_FILE);
	int NBAIC, EBTS;
	size_data >> NBAIC >> EBTS;
	data_table[states - 1][events - 1].update(NBAIC, EBTS, runtime);
}

void Datum::update(int NBAIC_up, int EBTS_up, double runtime_up) {
	NBAIC_size += NBAIC_up;
	EBTS_size += EBTS_up;
	runtime += runtime_up;
} 

void Datum::average() {
	NBAIC_size /= NUM_ITERATIONS;
	EBTS_size /= NUM_ITERATIONS;
	runtime /= NUM_ITERATIONS;
}

bool no_unfolds() {
	static bool first_iteration = true;
	if (first_iteration) {
		first_iteration = false;
		cout << "First Iteration\n"; 
		return true;
	}

	int unfolds;
	ifstream unfolds_in("./test/scalability_test/results/unfolds.txt");
	if (!unfolds_in.is_open()) {
		cout << "It's not there yo!\n";
		return true;
	}
	unfolds_in >> unfolds;
	unfolds_in.close();
	cout << unfolds << '\n';

	return unfolds == 0;
}