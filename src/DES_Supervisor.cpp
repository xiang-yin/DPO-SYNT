///////////////////////////////////////////////////////////////////////////////
//////////////DES Supervisor Application for BSCOPNBMAX and MPO////////////////
///////////////////////////////////////////////////////////////////////////////

#include <sstream>
#include <getopt.h>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <string.h>
#include "../include/UBTS.h"
#include "../include/DBTS.h"
#include "../include/CSR.h"
#include "../include/double_fsm.h"
#include "../include/supervisor.h"
#include "../include/Utilities.h"
using namespace std;

#ifdef DEBUG
ostream& out = cout;
#else
ostringstream oss;
ostream& out = oss;
#endif

Mode MODE_FLAG = BSCOPNBMAX;
bool MPO_CONDITION_FLAG = false;
bool VERBOSE_FLAG = false;
bool FILE_OUT_FLAG = false;

const char* const INITIAL_CLEAN_UP = "rm -f ./results/*";
string FSM_FSM_FILE = "./results/FSM.fsm";
string REQ_FSM_FILE = "./results/REQ_FSM.fsm";
string RESULT_FSM_FILE = "./results/RESULT_FSM.fsm";
string FSM_TXT_FILE = "./results/FSM.txt";
string A_UxG_FILE = "./results/A_UxG.fsm";
string A_UxG_REDUCED_FILE = "./results/A_UxG_reduced.fsm";
string NBAIC_FILE = "./results/NBAIC.fsm";
string ICS_FILE = "./results/ICS.fsm";
string UBTS_FILE = "./results/UBTS.fsm";
string EBTS_FILE = "./results/EBTS.fsm";
string MPO_FILE = "./results/MPO.fsm";
string BDO_FILE = "./results/BDO.fsm";
string MPRCP_FILE = "./results/MPRCP.fsm";

void do_BSCOPNBMAX(const string& FSM_file, const string& property,
				   const string& ISP_file);
void do_MPO(const string& FSM_file, const string& property,
		    const string& ISP_file);
void do_MPRCP(const string& FSM_file, const string& property,
		    const string& ISP_file, const string& required_property);
void getstrictsub_auto(FSM* fsm,FSM* req_fsm, IS_Property* isp);
void convert_fsm(const string& FSM_file);
void generate_supervisor(NBAIC* nbaic, FSM* fsm);
void generate_activation_policy(NBAIC* nbaic, FSM* fsm);
void generate_aic(NBAIC* nbaic, FSM* fsm);
void print_help();
void print_size_info();
void print_size_info(int NBAIC_size, int EBTS_size);
void write_unfolds(int num_unfolds);
void display_prompts(string& FSM_file, string& ISP_file, string& property, string& required_property);
void generate_result_fsm(FSM* ans_fsm, SUPV* sup, FSM* fsm);
void getsafety(FSM* fsm, FSM* safety_fsm, IS_Property* isp);
D_FSM* cross_product(FSM* fsm_1, FSM* fsm_2);


///////////////////////////////////////////////////////////////////////////////


int main(int argc, char* argv[]) {
	if (argc == 1) MODE_FLAG = INTERACTIVE;
	system(INITIAL_CLEAN_UP);
	static struct option long_options[] = {
		{"mode", required_argument, NULL, 'm'},
		{"MPO_condition", required_argument, NULL, 'c'},
		{"FSM_file", required_argument, NULL, 'f'},
		{"property", required_argument, NULL, 'p'},
		{"required_property", required_argument, NULL, 'r'},
		{"ISP_file", required_argument, NULL, 'i'},
		{"verbose", no_argument, NULL, 'v'},
		{"write_to_file", no_argument, NULL, 'w'},
		{"help", no_argument, NULL, 'h'},
		{0, 0, 0, 0}
	};

	char pause;
	int c, index = 0;
	string FSM_file, ISP_file, property, required_property;
	while ((c = getopt_long(argc, argv, "m:c:f:p:r:i:vwh", long_options, &index)) != -1) {
		switch (c) {
			case 'm':
				if (optarg) make_lower(optarg);
				if (strcmp(optarg, "mpo") == 0) MODE_FLAG = MPO;
				else if (strcmp(optarg, "bscopnbmax") == 0) MODE_FLAG = BSCOPNBMAX;
				else if (strcmp(optarg, "convert") == 0) MODE_FLAG = CONVERT;
				else if (strcmp(optarg, "interactive") == 0) MODE_FLAG = INTERACTIVE;
				else if (strcmp(optarg, "mprcp") == 0) MODE_FLAG = MPRCP;
				else if (VERBOSE_FLAG)
					cerr << "Error: " << optarg << " is not a valid mode."
						 << "Using default mode bscopnbmax.\n";
				break;
			case 'c':
				make_lower(optarg);
				if (strcmp(optarg, "max") == 0) MPO_CONDITION_FLAG = true;
				else if (strcmp(optarg, "min") == 0) MPO_CONDITION_FLAG = false;
				else if (VERBOSE_FLAG)
					cerr << "Error: " << optarg
						 << " is not a valid MPO condition."
						 << " Using default MPO condition min.\n";
				break;
			case 'f':
				FSM_file = optarg;
				break;
			case 'p':
				make_lower(optarg);
				property = optarg;
				break;
			case 'i':
				ISP_file = optarg;
				break;
			case 'r':
				required_property = optarg;
				break;
			case 'v':
				VERBOSE_FLAG = true;
				break;
			case 'w':
				FILE_OUT_FLAG = true;
				break;
			case '?':
				cerr << "Error: command " << c
					 << " is not defined. Printing help menu and exiting\n";
			case 'h':
				print_help();
				cin.get();
				return 0;
		}
	}
	if (MODE_FLAG == INTERACTIVE) display_prompts(FSM_file, ISP_file, property, required_property);
	if (MODE_FLAG == BSCOPNBMAX) do_BSCOPNBMAX(FSM_file, property, ISP_file);
	else if (MODE_FLAG == MPO) do_MPO(FSM_file, property, ISP_file);
	else if (MODE_FLAG == CONVERT) convert_fsm(FSM_file);
	else if (MODE_FLAG == MPRCP) do_MPRCP(FSM_file, property, ISP_file, required_property);

#ifndef DEBUG
	cout << oss.str();
#endif

	cout << "Press any key to continue...";
	cin >> pause;
	cout << '\n';
	return 0;
}


///////////////////////////////////////////////////////////////////////////////


/* Basic Supervisory Control and Observation Problem: Non-blocking
and Maximally Permissive Case */
void do_BSCOPNBMAX(const string& FSM_file, const string& property,
				   const string& ISP_file) {
	FSM* fsm = new FSM(FSM_file);
	IS_Property* isp = get_ISP(property, ISP_file,
							   fsm->states.regular, VERBOSE_FLAG);
	NBAIC* nbaic = new NBAIC(fsm, isp, out);
	if (!nbaic->is_empty()) generate_supervisor(nbaic, fsm);
	else if (VERBOSE_FLAG)
		out << "No maximally permissive supervisor exists for this FSM\n";
	delete fsm;
	delete isp;
	delete nbaic;
}

/* Most Permissive Observer */
void do_MPO(const string& FSM_file, const string& property,
		    const string& ISP_file) {
	FSM* fsm = new FSM(FSM_file, MODE_FLAG);
	IS_Property* isp = get_ISP(property, ISP_file,
							   fsm->states.regular, VERBOSE_FLAG);
	NBAIC* nbaic = new NBAIC(fsm, isp, out, MODE_FLAG);
	if (!nbaic->is_empty()) generate_activation_policy(nbaic, fsm);
	else if (VERBOSE_FLAG)
		out << "No " << (MPO_CONDITION_FLAG ? "maximal" : "minimal")
			<< " activation policy exists for this FSM\n";
	else write_unfolds(0);
	delete fsm;
	delete isp;
	delete nbaic;
}

void do_MPRCP(const string& FSM_file, const string& property,
		    const string& ISP_file, const string& required_property) {
	FSM* fsm = new FSM(FSM_file, MODE_FLAG);
	FSM* req_fsm = new FSM(required_property, MODE_FLAG);
	FSM* safety_fsm = new FSM(ISP_file, MODE_FLAG);
	IS_Property* isp = get_ISP("","", fsm->states.regular, VERBOSE_FLAG);

	getsafety(fsm, safety_fsm, isp);
	getstrictsub_auto(fsm, req_fsm, isp);
	
	NBAIC* nbaic = new NBAIC(fsm, isp, out, MODE_FLAG);
	if (FILE_OUT_FLAG){
		ofstream fsm_file_out(FSM_FSM_FILE.c_str());
		fsm->print_fsm(fsm_file_out);
		fsm_file_out.close();
		ofstream req_file_out(REQ_FSM_FILE.c_str());
		req_fsm->print_fsm(req_file_out);
		req_file_out.close();
	}
	if (VERBOSE_FLAG){
		out<<"FSM: \n";
		fsm->print_txt(out);
		out<<"REQ_FSM: \n";
		req_fsm->print_txt(out);
	}
	if (!nbaic->is_empty()) generate_aic(nbaic, fsm);
	else if (VERBOSE_FLAG)
			out << "No maximally permissive supervisor exists for this FSM\n";
	else write_unfolds(0);
	if (!nbaic->is_empty()){
		bool sol = false;
		DBTS* T_r = new DBTS(nbaic, req_fsm, out, sol);// get T_R
		if (sol){
			CSR* csr = new CSR(T_r, out);// get Control simulation relation
			
			SUPV* sup = new SUPV(csr);// get the result supervisor
			if (VERBOSE_FLAG){
				out<<"DBTS for T_R:\n";
				T_r->print();
				csr->print();
				out<< "RESULT SUPERVISOR:\n";
				sup->print();
			}
			FSM* ans_fsm = new FSM(0,0);
			generate_result_fsm(ans_fsm, sup, fsm);
			if (VERBOSE_FLAG){
				out << "RESULT FSM:\n";
				ans_fsm->print_txt(out);
			}
			if (FILE_OUT_FLAG){
				ofstream ans_file_out(RESULT_FSM_FILE.c_str());
				ans_fsm->print_fsm(ans_file_out);
				ans_file_out.close();
				nbaic->print_fsm(NBAIC_FILE.c_str());
				sup->print(MPRCP_FILE.c_str());
			}
			delete ans_fsm;
			delete sup;
			delete csr;
		}
		delete T_r;
	}
	else
		out << "No Solution, no AIC!"<<endl;
	delete fsm;
	delete isp;
	delete nbaic;
}

/* Finite State Machine file conversion utility */
void convert_fsm(const string& FSM_file) {
	FSM* fsm = new FSM(FSM_file);
	/* File extension is .fsm--convert to .txt format */
	if (FSM_file.find(".fsm") != string::npos) {
		if (VERBOSE_FLAG) fsm->print_txt(out);
		if (FILE_OUT_FLAG) {
			ofstream file_out(FSM_TXT_FILE.c_str());
			fsm->print_txt(file_out);
			file_out.close();
		}
	}
	/* File extension is not .fsm--convert to .fsm format */
	else {
		if (VERBOSE_FLAG) fsm->print_fsm(out);
		if (FILE_OUT_FLAG) {
			ofstream file_out(FSM_FSM_FILE.c_str());
			fsm->print_fsm(file_out);
			file_out.close();
		}
	}
}

void generate_supervisor(NBAIC* nbaic, FSM* fsm) {
	/* Build the inital unfolded bipartite transition system */
	UBTS ubts(nbaic, out);
	ubts.expand();
	/* Build the ics representation of our ubts */
	ICS ics(ubts, fsm, out);
	if (VERBOSE_FLAG) {
		nbaic->print();
		ubts.print();
		ics.print();
	}
	int num_unfolds = 0;
	/* Enters loop if there exists a state that is not coaccessible */
	while (ICS_STATE* entrance_state = ics.get_entrance_state(ubts)) {
		if (VERBOSE_FLAG) entrance_state->print(out);
		/* Build the live decision string from the
		entrance state to a marked state*/
		LDS lds(out, nbaic, entrance_state);
		lds.compute_maximal();
		/* Add transitions in the live decision string to our ubts */
		ubts.augment(lds);
		ubts.expand();
		/* Rebuild ICS for new ubts */
		ics = ICS(ubts, fsm, out);
		if (VERBOSE_FLAG) {
			lds.print();
			ubts.print();
			ics.print();
		}
		++num_unfolds;
	}
	
	if (VERBOSE_FLAG) ubts.print();
	ofstream file_out(A_UxG_FILE.c_str());
	ics.print_A_UxG(ubts, file_out, FILE_OUT_FLAG, VERBOSE_FLAG);
	file_out.close();
	if (FILE_OUT_FLAG) {
		fsm->print_fsm(FSM_FSM_FILE.c_str());
		nbaic->print_fsm(NBAIC_FILE.c_str());
		ubts.print(UBTS_FILE.c_str(), false);
		ubts.print(EBTS_FILE.c_str(), true);
		ics.print_fsm(ICS_FILE.c_str());
		ics.reduce_A_UxG(A_UxG_FILE.c_str(), A_UxG_REDUCED_FILE.c_str());
	}
}

void generate_activation_policy(NBAIC* nbaic, FSM* fsm) {
	if (VERBOSE_FLAG) nbaic->print();
	if (FILE_OUT_FLAG) nbaic->print_fsm(MPO_FILE.c_str());
	nbaic->reduce_MPO(MPO_CONDITION_FLAG);
	if (VERBOSE_FLAG) nbaic->print(true);
	if (FILE_OUT_FLAG) nbaic->print_fsm(BDO_FILE.c_str());
	if (FILE_OUT_FLAG) fsm->print_fsm(FSM_FSM_FILE.c_str());
}

void generate_aic(NBAIC* nbaic, FSM* fsm){
	if (VERBOSE_FLAG) nbaic->print();
	if (FILE_OUT_FLAG)  nbaic->print_fsm(NBAIC_FILE.c_str());
}

void print_help() {
	cout << "DES Supervisor Application for BSCOPNBMAX and MPO \n"
		 << "Controls:\n"
		 << "\tMode [-m] - switch between the [INTERACTIVE], [BSCOPNBMAX], [MPO], and [CONVERT] modes\n"
		 << "\tMPO_condition [-c] - request the MPO to find a [min]imal or [max]imal solution\n"
		 << "\tFSM_file [-f] - provide an FSM file for processing\n"
		 << "\tProperty [-p] - provide an implemented information state property\n"
		 << "\tISP_file [-i] - provide a corresponding file for the specified ISP property\n"
		 << "\tVerbose [-v] - request more detailed output\n"
		 << "\tWrite_to_File [-w] - write the UBTS, EBTS, NBAIC, and A_UxG to separate .fsm files in the ./results folder\n"
		 << "\tHelp [-h] - display help menu\n"
		 << "For more information, please see the README document\n" << flush;
}

void display_prompts(string& FSM_file, string& ISP_file, string& property, string& required_property) {
	string arg_str, arg_str_lower;
	cout << "*********************DES Supervisor Application***********************\n"
		 << endl;
	bool no_valid_argument = true;
	while (no_valid_argument) {
		cout << "Please select a mode for program execution [BSCOPNBMAX | MPO | MPRCP | CONVERT]: "
			 << flush;
		cin >> arg_str;
		cout << endl;
		arg_str_lower.resize(arg_str.size());
		transform(arg_str.begin(), arg_str.end(), arg_str_lower.begin(), ::tolower);
		switch (arg_str_lower[2]) {
			case 'c':
				if (arg_str_lower == "bscopnbmax")
					no_valid_argument = false;
				break;
			case 'o':
				if (arg_str_lower == "mpo") {
					MODE_FLAG = MPO;
					no_valid_argument = false;
				}
				break;
			case 'n':
				if (arg_str_lower == "convert") {
					MODE_FLAG = CONVERT;
					no_valid_argument = false;
				}
				break;
			case 'r':
				if (arg_str_lower == "mprcp"){
					MODE_FLAG = MPRCP;
					no_valid_argument = false;
				}
		}
		if (no_valid_argument) cout << "Error reading mode type " << arg_str << endl;
	}
	if (MODE_FLAG == MPO) {
		no_valid_argument = true;
		while (no_valid_argument) {
			cout << "Would you like to synthesize a minimal or maximal "
				 << "sensor activation policy? [MIN | MAX]: " << flush;
			cin >> arg_str;
			cout << endl;
			arg_str_lower.resize(arg_str.size());
			transform(arg_str.begin(), arg_str.end(), arg_str_lower.begin(), ::tolower);
			switch (arg_str_lower[1]) {
				case 'i':
					if (arg_str_lower == "min")
						no_valid_argument = false;
					break;
				case 'a':
					if (arg_str_lower == "max") {
						MPO_CONDITION_FLAG = true;
						no_valid_argument = false;
					}
					break;
			}
			if (no_valid_argument) cout << "Error reading MPO condition " << arg_str << endl;
		}
	}
	no_valid_argument = true;
	while (no_valid_argument) {
		cout << "Please enter the FSM file you would like to process: " << flush;
		cin >> FSM_file;
		cout << endl;
		ifstream file_open_test(FSM_file.c_str());
		if (file_open_test.is_open()) no_valid_argument = false;
		else cout << "Error: file " << FSM_file << " could not be opened" << endl;
		file_open_test.close();
	}
	if (MODE_FLAG != CONVERT) {
		no_valid_argument = true;
		bool using_ISP = false;
		while (no_valid_argument) {
			cout << "Would you like to use an inforamtion state property? [y | n]: "
				 << flush;
			cin >> arg_str;
			cout << endl;
			if (tolower(arg_str[0]) == 'n') no_valid_argument = false;
			else if (tolower(arg_str[0] != 'y')) continue;
			else {
				no_valid_argument = false;
				using_ISP = true;
			}
		}
		if (using_ISP) {
			no_valid_argument = true;
			while (no_valid_argument) {
				cout << "Please enter which property you would like "
					 << "to use [SAFETY | OPACITY | DISAMBIGUATION]: " << flush;
				cin >> arg_str;
				cout << endl;
				property.resize(arg_str.size());
				transform(arg_str.begin(), arg_str.end(), property.begin(), ::tolower);
				if (property == "safety" || property == "opacity" ||
					property == "disambiguation")
					no_valid_argument = false;
			}
			no_valid_argument = true;
			while (no_valid_argument) {
				cout << "Please enter the information state "
					 << "property file you would like to use: " << flush;
				cin >> ISP_file;
				cout << endl;
				ifstream file_open_test(ISP_file);
				if (file_open_test.is_open()) no_valid_argument = false;
				else cout << "Error: file " << ISP_file << " could not be opened" << endl;
				file_open_test.close();
			}
		}
	}
	if (MODE_FLAG == MPRCP){
		no_valid_argument = true;
		while (no_valid_argument) {
			cout << "Please enter the required "
				 << "property file you would like to use: " << flush;
				cin >> required_property;
				cout << endl;
				ifstream file_open_test(required_property);
				if (file_open_test.is_open()) no_valid_argument = false;
				else cout << "Error: file " << required_property << " could not be opened" << endl;
				file_open_test.close();
		}
	}
	no_valid_argument = true;
	while (no_valid_argument) {
		cout << "Would you like to turn on console output? "
			 << "This is not recommended for large inputs [y | n]: " << flush;
		cin >> arg_str;
		cout << endl;
		if (tolower(arg_str[0]) == 'n') no_valid_argument = false;
		else if (tolower(arg_str[0] != 'y')) continue;
		else {
			no_valid_argument = false;
			VERBOSE_FLAG = true;
		}
	}
	no_valid_argument = true;
	while (no_valid_argument) {
		cout << "Would you like to turn on file output? [y | n]: " << flush;
		cin >> arg_str;
		cout << endl;
		if (tolower(arg_str[0]) == 'n') no_valid_argument = false;
		else if (tolower(arg_str[0] != 'y')) continue;
		else {
			no_valid_argument = false;
			FILE_OUT_FLAG = true;
		}
	}
	if (FILE_OUT_FLAG){
		no_valid_argument = true;
		while (no_valid_argument) {
			cout << "Would you like to self-defined where you want the file output located? [y | n]: " << flush;
			cin >> arg_str;
			cout << endl;
			if (tolower(arg_str[0]) == 'n') no_valid_argument = false;
			else if (tolower(arg_str[0] != 'y')) continue;
			else {
				no_valid_argument = false;
				if (MODE_FLAG == BSCOPNBMAX){
					cout<< "Input the path where you want to output the FSM_FSM_FILE: "<< endl;
					cin >> FSM_FSM_FILE;
					cout<< "Input the path where you want to output the NBAIC_FILE: "<<endl;
					cin>> NBAIC_FILE;
					cout<< "Input the path where you want to output the UBTS_FILE: "<<endl;
					cin>>UBTS_FILE;
					cout<< "Input the path where you want to output the EBTS_FILE: "<<endl;
					cin>>EBTS_FILE;
					cout<< "Input the path where you want to output the ICS_FILE: "<<endl;
					cin>>ICS_FILE;
					cout<< "Input the path where you want to output the A_UxG_FILE: "<<endl;
					cin>>A_UxG_FILE;
					cout<< "Input the path where you want to output the A_UxG_REDUCED_FILE: "<<endl;
					cin>>A_UxG_REDUCED_FILE;
				}
				if (MODE_FLAG == MPO){
					cout<< "Input the path where you want to output the FSM_FSM_FILE: "<<endl;
					cin>>FSM_FSM_FILE;
					cout<< "Input the path where you want to output the MPO_FILE: "<<endl;
					cin>>MPO_FILE;
					cout<< "Input the path where you want to output the BDO_FILE: "<<endl;
					cin>>BDO_FILE;
				}
				if (MODE_FLAG == MPRCP){
					cout<< "Input the path where you want to output the FSM_FSM_FILE: "<<endl;
					cin>>FSM_FSM_FILE;
					cout<< "Input the path where you want to output the REQ_FSM_FILE: "<<endl;
					cin>>REQ_FSM_FILE;
					cout<< "Input the path where you want to output the RESULT_FSM_FILE: "<<endl;
					cin>>RESULT_FSM_FILE;
					cout<< "Input the path where you want to output the NBAIC_FILE: "<<endl;
					cin>>NBAIC_FILE;
					cout<< "Input the path where you want to output the MPRCP_FILE: "<<endl;
					cin>>MPRCP_FILE;
				}
			}
		}
	}
	cout << "Executing program..." << endl;
}

void write_unfolds(int num_unfolds) {
	ofstream unfold_out("./test/scalability_test/results/unfolds.txt");
	unfold_out << num_unfolds;
	unfold_out.close();
}

void getsafety(FSM* fsm, FSM* safety_fsm, IS_Property* isp){
	//get safety states from the safety fsm input K.
	FSM* fsm_A = new FSM(safety_fsm->nstates+1, safety_fsm->nevents);
	fsm_A->transitions = safety_fsm->transitions;
	fsm_A->states.regular = safety_fsm->states.regular;
	fsm_A->states.inverse = safety_fsm->states.inverse;
	fsm_A->marked = safety_fsm->marked;
	fsm_A->events.regular = safety_fsm->events.regular;
	fsm_A->events.inverse = safety_fsm->events.inverse;
	fsm_A->controllable = safety_fsm->controllable;
	fsm_A->observable = safety_fsm->observable;
	fsm_A->uu = safety_fsm->uu;
	fsm_A->uo = safety_fsm->uo;
	fsm_A->states.regular["dead"] = safety_fsm->nstates;
	fsm_A->states.inverse[safety_fsm->nstates] = "dead";
	std::unordered_map<EVENT, STATE> new_map;
	for (STATE i = 0; i<safety_fsm->nstates; i++){
		std:unordered_map<EVENT, STATE> new_mapp = fsm_A->transitions[i];
		for (EVENT j = 0; j<safety_fsm->nevents; j++)
			if (fsm_A->transitions[i].find(j) == fsm_A->transitions[i].end()){
				new_mapp[j] = safety_fsm->nstates;
			}
		fsm_A->transitions[i] = new_mapp;
	}
	for (EVENT j = 0; j<safety_fsm->nevents; j++)
		new_map[j] = safety_fsm->nstates;
	fsm_A->transitions.push_back(new_map);
	D_FSM* dfsm_GA = cross_product(fsm, fsm_A);
	fsm->transitions = dfsm_GA->transitions;
	fsm->nstates = dfsm_GA -> nstates;
	fsm->marked = dfsm_GA->marked;
	std::unordered_map<STATE, std::string> temp = fsm->states.inverse;
	fsm->states.regular.clear();
	fsm->states.inverse.clear();
	std::vector<bool> unsafe_states;
	Safety* safe = dynamic_cast<Safety*>(isp);
	for (STATE i=0; i<dfsm_GA->nstates;i++){
		D_STATE* ds = dfsm_GA->states.inverse[i];
		if (ds->state_2 == safety_fsm->nstates)
			safe->unsafe_states.push_back(true);
		else
			safe->unsafe_states.push_back(false);
		fsm->states.regular["("+temp[ds->state_1] + "," + fsm_A->states.inverse[ds->state_2]] = i;
		fsm->states.inverse[i] = "(" + temp[ds->state_1] + "," + fsm_A->states.inverse[ds->state_2];
	}
}

void getstrictsub_auto(FSM* fsm,FSM* req_fsm, IS_Property* isp){
	//transfer req_fsm into fsm's strict sub automata.
	FSM* fsm_A = new FSM(req_fsm->nstates+1, req_fsm->nevents);
	fsm_A->transitions = req_fsm->transitions;
	fsm_A->states.regular = req_fsm->states.regular;
	fsm_A->states.inverse = req_fsm->states.inverse;
	fsm_A->marked = req_fsm->marked;
	fsm_A->events.regular = req_fsm->events.regular;
	fsm_A->events.inverse = req_fsm->events.inverse;
	fsm_A->controllable = req_fsm->controllable;
	fsm_A->observable = req_fsm->observable;
	fsm_A->uu = req_fsm->uu;
	fsm_A->uo = req_fsm->uo;
	fsm_A->states.regular["dead"] = req_fsm->nstates;
	fsm_A->states.inverse[req_fsm->nstates] = "dead";
	std::unordered_map<EVENT, STATE> new_map;
	for (STATE i = 0; i<req_fsm->nstates; i++){
		std:unordered_map<EVENT, STATE> new_mapp = fsm_A->transitions[i];
		for (EVENT j = 0; j<req_fsm->nevents; j++)
			if (fsm_A->transitions[i].find(j) == fsm_A->transitions[i].end()){
				new_mapp[j] = req_fsm->nstates;
			}
		fsm_A->transitions[i] = new_mapp;
	}
	for (EVENT j = 0; j<req_fsm->nevents; j++)
		new_map[j] = req_fsm->nstates;
	fsm_A->transitions.push_back(new_map);
	D_FSM* dfsm_GA = cross_product(fsm, fsm_A);
	fsm->transitions = dfsm_GA->transitions;
	fsm->nstates = dfsm_GA -> nstates;
	fsm->marked = dfsm_GA->marked;
	std::unordered_map<STATE, std::string> temp = fsm->states.inverse;
	fsm->states.regular.clear();
	fsm->states.inverse.clear();
	std::vector<bool> unsafe_states;
	Safety* safe = dynamic_cast<Safety*>(isp);
	for (STATE i=0; i<dfsm_GA->nstates;i++){
		D_STATE* ds = dfsm_GA->states.inverse[i];
		if (safe->unsafe_states[ds->state_1])
			unsafe_states.push_back(true);
		else
			unsafe_states.push_back(false);
		fsm->states.regular[temp[ds->state_1] + "," + fsm_A->states.inverse[ds->state_2] + ")"] = i;
		fsm->states.inverse[i] = temp[ds->state_1] + "," + fsm_A->states.inverse[ds->state_2] + ")";
	}

	safe->unsafe_states = unsafe_states;
	req_fsm->nstates = 0;
	req_fsm->transitions.clear();
	req_fsm->marked.clear();
	std::unordered_map<STATE, std::string> tmp = req_fsm->states.inverse;
	req_fsm->states.regular.clear();
	req_fsm->states.inverse.clear();
	std::unordered_map<STATE, STATE> new_old_map;

	for (STATE i = 0; i<dfsm_GA->nstates; i++){
		D_STATE* ds = dfsm_GA->states.inverse[i];
		if (ds->state_2 != fsm_A->nstates-1){
			req_fsm->states.regular[temp[ds->state_1] + "," + tmp[ds->state_2] + ")"] = req_fsm->nstates;
			req_fsm->states.inverse[req_fsm->nstates] = temp[ds->state_1] + "," + tmp[ds->state_2] + ")";

			new_old_map[i] = req_fsm->nstates;
			req_fsm->transitions.push_back(dfsm_GA->transitions[i]);

			req_fsm->nstates++;
		}
	}

	for (STATE i = 0; i<req_fsm->nstates; i++){
		std::unordered_map<EVENT, STATE> new_transition;
		for (auto j:req_fsm->transitions[i]){
			if (new_old_map.find(j.second) != new_old_map.end())
				new_transition[j.first] = new_old_map[j.second];
		}
		req_fsm->transitions[i] = new_transition;
	}
}

D_FSM* cross_product(FSM* fsm_1, FSM* fsm_2){
	//get the cross product of fsm_1 and fsm_2.
	D_FSM* dfsm = new D_FSM();
	dfsm->fsm_1 = fsm_1;
	dfsm->fsm_2 = fsm_2;
	D_STATE* init = new D_STATE;
	dfsm->nevents = fsm_1->nevents;
	dfsm->events.regular = fsm_1->events.regular;
	dfsm->events.inverse = fsm_1->events.inverse;
	dfsm->controllable = fsm_1->controllable;
	dfsm->observable = fsm_1->observable;
	dfsm->uu = fsm_1->uu;
	dfsm->uo = fsm_1->uo;
	init->state_1 = 0;
	init->state_2 = 0;
	dfsm->dstate_map[0][0] = init;
	dfsm->states.regular[init] = 0;
	dfsm->states.inverse[0] = init;
	std::queue<STATE> search_queue;
	search_queue.push(0);
	dfsm->nstates = 1;
	while (!search_queue.empty()){
		STATE current = search_queue.front();
		search_queue.pop();
		D_STATE* cur_ds = dfsm->states.inverse[current];
		std::unordered_map<EVENT, STATE> new_map;
		for (auto i: fsm_1->transitions[cur_ds->state_1]){
			if (fsm_2->transitions[cur_ds->state_2].find(i.first) != fsm_2->transitions[cur_ds->state_2].end()){
				if (dfsm->dstate_map.find(i.second) == dfsm->dstate_map.end() 
					|| dfsm->dstate_map[i.second].find(fsm_2->transitions[cur_ds->state_2][i.first]) == dfsm->dstate_map[i.second].end()){
					D_STATE* new_ds = new D_STATE;
					new_ds->state_1 = i.second;
					new_ds->state_2 = fsm_2->transitions[cur_ds->state_2][i.first];
					search_queue.push(dfsm->nstates);
					dfsm->dstate_map[new_ds->state_1][new_ds->state_2] = new_ds;
					dfsm->states.regular[new_ds] = dfsm->nstates;
					dfsm->states.inverse[dfsm->nstates] = new_ds;
					dfsm->nstates++;
				}
				D_STATE* ds = dfsm->dstate_map[i.second][fsm_2->transitions[cur_ds->state_2][i.first]];
				new_map[i.first] = dfsm->states.regular[ds];
			}
		}
		dfsm->transitions.push_back(new_map);
	}
	dfsm->marked.resize(dfsm->nstates, true);
	return dfsm;
}

void generate_result_fsm(FSM* ans_fsm, SUPV* sup, FSM* fsm){
	//generate the result fsm from the supervisor.
	ans_fsm->nevents = fsm->nevents;
	ans_fsm->events.regular = fsm->events.regular;
	ans_fsm->events.inverse = fsm->events.inverse;
	ans_fsm->controllable = fsm->controllable;
	ans_fsm->observable = fsm->observable;

	for (auto z_dbts:sup->ZSL){
		ZS* zs = z_dbts->get_ZS();
		for (STATE i = 0; i<fsm->nstates; i++){
			if ((zs->IS[i])
				&&(!ans_fsm->states.find_value(fsm->states.get_key(i)))){
				string temp = fsm->states.get_key(i);
				ans_fsm->states.regular[temp] = ans_fsm->nstates;
				ans_fsm->states.inverse[ans_fsm->nstates] = temp;
				ans_fsm->nstates++;
			}
		}
	}
	ans_fsm->marked.resize(ans_fsm->nstates,true);
	for (STATE i = 0; i<ans_fsm->nstates; i++){
		STATE org_state = fsm->states.regular[ans_fsm->states.inverse[i]];
		std::unordered_map<EVENT, STATE> new_map;
		for (EVENT e = 0; e<ans_fsm->nevents; e++){
			if ((!fsm->controllable[e])
				&&(fsm->transitions[org_state].find(e)!= fsm->transitions[org_state].end()))
				new_map[e] = ans_fsm->states.regular[fsm->states.inverse[fsm->transitions[org_state][e]]];
		}
		ans_fsm->transitions.push_back(new_map);
	}

	for (auto z_dbts:sup->ZSL){
		ZS* zs = z_dbts->get_ZS();
		for (STATE i = 0; i<fsm->nstates; i++){
			if (zs->IS[i]){
				STATE ans_state = ans_fsm->states.regular[fsm->states.inverse[i]];
				for (EVENT e = 0; e<ans_fsm->nevents; e++){
					if ((zs->CD[e])
						&&(fsm->transitions[i].find(e)!= fsm->transitions[i].end())){
						ans_fsm->transitions[ans_state][e] = ans_fsm->states.regular[fsm->states.inverse[fsm->transitions[i][e]]];
					}
				}
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
