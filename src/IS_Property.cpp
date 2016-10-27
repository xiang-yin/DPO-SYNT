#include <cstring>
#include <cstdlib>
#include <iostream>
#include "../include/IS_Property.h"
#include "../include/Utilities.h"
using namespace std;

static void read_state_file(const string& filename, vector<bool>& state_subset,
					 		unordered_map<string, STATE>& all_states);
static void move_file_line(ifstream& file_in, const char* const filename);


///////////////////////////////////////////////////////////////////////////////


Opacity::Opacity(const vector<bool>& secret_states_)
	: secret_states(secret_states_) {}
Opacity::Opacity(const string& filename,
				 unordered_map<string, STATE>& all_states) {
	read_state_file(filename, secret_states, all_states);
}

bool Opacity::operator() (const INFO_STATE& IS) const {
	for (STATE s = 0; s < secret_states.size(); ++s)
		/* At least one state in IS is not in secret_states */
		if (IS[s] && !secret_states[s]) return true;
	/* All states are in secret_states */
	return false;
}


///////////////////////////////////////////////////////////////////////////////


Safety::Safety() {}
Safety::Safety(const vector<bool>& unsafe_states_)
	: unsafe_states(unsafe_states_) {}
Safety::Safety(const string& filename,
			   unordered_map<string, STATE>& all_states) {

	read_state_file(filename, unsafe_states, all_states);
}

bool Safety::operator() (const INFO_STATE& IS) const {
	for (STATE s = 0; s < unsafe_states.size(); ++s)
		/* At least one state is unsafe */
		if (IS[s] && unsafe_states[s]) return false;
	/* All states are safe */
	return true;
}


///////////////////////////////////////////////////////////////////////////////


Disambiguation::Disambiguation(const std::vector<bool>& A_states_,
			  		 		   const std::vector<bool>& B_states_)
	: A_states(A_states_), B_states(B_states_) {}
Disambiguation::Disambiguation(const string& filename,
			   		 		   unordered_map<string, STATE>& all_states) {
	ifstream state_file(filename.c_str());
	const char* const A_file = "./test/A_states.txt";
	const char* const B_file = "./test/B_states.txt";
	const string remove = "rm -f ";
	/* Use getline to seperate A and B into 2 files */
	move_file_line(state_file, A_file);
	move_file_line(state_file, B_file);
	/* Read each line into seperate vector */
	read_state_file(A_file, A_states, all_states);
	read_state_file(B_file, B_states, all_states);
	/* Remove partitioned files from directory */
	string rm_A(remove + A_file);
	string rm_B(remove + B_file);
	system(rm_A.c_str());
	system(rm_B.c_str());
}

void move_file_line(ifstream& file_in, const char* const filename) {
	string file_line;
	ofstream file_out(filename);
	if (getline(file_in, file_line)) file_out << file_line;
	file_out.close();
}

bool Disambiguation::operator() (const INFO_STATE& IS) const {
	bool has_A = false, has_B = false;
	for (STATE s = 0; s < IS.size(); ++s) {
		if (!IS[s]) continue;
		/* State is in IS and A_states list */
		if (A_states[s]) {
			has_A = true;
			/* IS uses states from both lists--violates distinction */
			if (has_B) return false;
		}
		/* State is in IS and B_states list */
		if (B_states[s]) {
			has_B = true;
			/* IS uses states from both lists--violates distinction */
			if (has_A) return false;
		}
	}
	/* IS does not contain states from both A_ and B_states--
	bipartition remains distinct */
	return true;
}


///////////////////////////////////////////////////////////////////////////////

IS_Property* get_ISP(const string& property, const string& ISP_file,
					 unordered_map<string, STATE>& states, bool verbose) {
	if (property.empty() || ISP_file.empty()) return new Safety();

	if (property == "opacity")
		return new Opacity(ISP_file, states);
	else if (property == "safety")
		return new Safety(ISP_file, states);
	else if (property == "disambiguation")
		return new Disambiguation(ISP_file, states);
	else {
		if (verbose)
			cerr << "Could not construct information state property from "
		    	 << "argument " << property << ". Using trivial ISP.\n";
		return new Safety();
	}
}

IS_Property* get_ISP() {
	return new Safety();
}

static void read_state_file(const string& filename, vector<bool>& state_subset,
					 		unordered_map<string, STATE>& all_states) {
	state_subset.resize(all_states.size());
	ifstream state_file(filename.c_str());
	if (!state_file.is_open()) {
		cerr << "Error: file \'" << filename << "\' could not be read\n";
		cin.get();
		exit(1);
	}

	string s;
	while (state_file >> s) {
		if (all_states.find(s) != all_states.end()) {
			state_subset[all_states[s]] = true;
		}
		else {
			cerr << "State " << s << "in file " << filename << " is not "
				 << "a valid state\n";
			cin.get();
			exit(1);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////
