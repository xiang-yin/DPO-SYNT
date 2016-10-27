#include <queue>
#include <unordered_set>
#include "../include/DBTS.h"
#include "../include/Utilities.h"
using namespace std;

DBTS::DBTS(NBAIC* aic_, FSM* req_fsm_, ostream& os_, bool& sol)
	: aic(aic_), os(os_), req_fsm(req_fsm_){
	/* Create initial Y-State */
	Y_DBTS* y0 = new Y_DBTS(aic->YSL[0]);
	YSL.push_back(y0);
	/* get required state of R in G */
	req_states.resize(aic->fsm->nstates,false);
	for (STATE i = 0;i < aic->fsm->nstates; i++){
		if (req_fsm->states.find_value(aic->fsm->states.inverse[i])){
			req_states[i] = true;
		}
	}
	sol = DoDFS(y0);

}

DBTS::~DBTS() {
	for (Y_DBTS* y_Dbts : YSL) delete y_Dbts;
	for (Z_DBTS* z_Dbts : ZSL) delete z_Dbts; 
}


bool DBTS_State::operator==(const DBTS_State& rhs) const {
	return nbs == rhs.nbs;
}

bool DBTS::DoDFS(Y_DBTS* y){
	bool have_sol = true;
	CONTROL_DECISION Act(aic->fsm->nevents, false);
	
	YS* y_state = y->get_YS();
	std::vector<STATE> y_req_states = get_states(y_state);

	if (!y_req_states.empty()){
		Act = get_cd(y_req_states);
		
		if (y_state->transition.find(Act) == y_state->transition.end()){
			os<<"No Solution"<<endl;
			return false;
		}
	}
	bool judge;
	ZS* z_state = y_state -> transition[Act];
	/* get the real z and judge whether the z state is already in the DBTS*/
	Z_DBTS* z = get_Z_DBTS(z_state, Act, judge);
	y->child = z;
	z->parents.push_back(y);

	if (!judge){
		ZSL.push_back(z);
		for (auto i:z_state->transition){
			bool judge_y;
			Y_DBTS* new_y = get_Y_DBTS(i.second, judge_y);

			z->children[i.first] = new_y;
			new_y->parents.push_back(z);
			if (!judge_y){
				YSL.push_back(new_y);
				have_sol = DoDFS(new_y);
			}
		}
	}
	return have_sol;	
}

std::vector<STATE> DBTS::get_states(YS* y){
	std::vector<STATE> result;
	std::unordered_set<STATE> state_set;
	std::queue<STATE> bfs;
	for (STATE i = 0; i < aic->fsm->nstates; i++){
		if (y->IS[i] && req_states[i]){
			bfs.push(i);
			state_set.insert(i);
			result.push_back(i);
		}
	}
	while (!bfs.empty()){
		STATE cur_state = bfs.front();
		bfs.pop();
		if (state_set.find(cur_state) == state_set.end()){
			state_set.insert(cur_state);
			result.push_back(cur_state);
		}
		STATE cur_req_state = req_fsm->states.regular[aic->fsm->states.inverse[cur_state]];
		for (auto i:req_fsm->transitions[cur_req_state]){
			STATE next_state = aic->fsm->states.regular[req_fsm->states.inverse[i.second]];
			if (!req_fsm->observable[i.first] && 
				state_set.find(next_state) == state_set.end()){
				state_set.insert(next_state);
				result.push_back(next_state);
				bfs.push(next_state);
			}
		}
	}
	return result;
}

CONTROL_DECISION DBTS::get_cd(const std::vector<STATE> y_req_states){
	CONTROL_DECISION result(aic->fsm->nevents, false);
	for (STATE i:y_req_states)
		for (auto j:aic->fsm->transitions[i])
			if (req_states[j.second] && aic->fsm->controllable[j.first]){
				result[j.first] = true;
			}
	return result;
}

Z_DBTS* DBTS::get_Z_DBTS(ZS* z, CONTROL_DECISION Act, bool& judge){
	judge = false;
	for (Z_DBTS* i:ZSL)
		if (z==i->get_ZS() && Act == i->CD){
			judge = true;
			return i;
		}
	return new Z_DBTS(z, Act);
}

Y_DBTS* DBTS::get_Y_DBTS(YS* y, bool& judge){
	judge = false;
	for (Y_DBTS* i:YSL)
		if (y==i->get_YS()){
			judge = true;
			return i;
		}
	return new Y_DBTS(y);;
}

void DBTS::print(const char* const filename) {
	ofstream file_out(filename);
	file_out << YSL.size() + ZSL.size()
			 << "\r\n\r\n";
	for (Y_DBTS* y_Dbts : YSL) {
		if (!y_Dbts->child) continue;
		file_out << "{" << get_subset_string(y_Dbts->get_YS()->IS,
											  aic->fsm->states) 
				 << "}	0	"
				 << (y_Dbts->child ? "1" : "0") << "\r\n";
		if (y_Dbts->child)
			file_out << '{' << get_subset_string(y_Dbts->child->CD,
												 aic->fsm->events) << "}	{{"
					 << get_subset_string(y_Dbts->child->get_ZS()->IS,
					 					  aic->fsm->states) << "},{"
					 << get_subset_string(y_Dbts->child->CD,
					 					  aic->fsm->events) 
					 << "}} 	c	o\r\n";
		file_out << "\r\n";
	}
	for (Z_DBTS* z_Dbts : ZSL) {
		file_out << "{{" << get_subset_string(z_Dbts->get_ZS()->IS,
											 aic->fsm->states) << "},{"
				 << get_subset_string(z_Dbts->CD, aic->fsm->events) 
				 << "}}	1	"
				 << z_Dbts->children.size() << "\r\n";
		for (auto& pair : z_Dbts->children)
			file_out << aic->fsm->events.get_key(pair.first) << "	{"
					 << get_subset_string(pair.second->get_YS()->IS,
					 					  aic->fsm->states) << "}	"
					 << (aic->fsm->controllable[pair.first] ? "c	" : "uc	")
					 << (aic->fsm->observable[pair.first] ? "o" : "uo")
					 << "\r\n";
		file_out << "\r\n";
	}
	file_out.close();
}

void DBTS::print() {
	os << "******************************************************************************\n"
	   << "********************Determined Bipartite Transition System********************\n"
	   << "******************************************************************************\n";
	if (YSL.empty()) os << "No maximally permissive supervisor exists for this FSM\n";
	else {
		os << "Y-States:\n";
		for (Y_DBTS* y_Dbts : YSL) {
			os << "\tY_DBTS State: {"
			   << get_subset_string(y_Dbts->get_YS()->IS, aic->fsm->states)
			   <<"}\n";
			if (y_Dbts->child)
				os << "\tTransitions to:\n"
				   << "\t\tZ_DBTS State {{"
				   << get_subset_string(y_Dbts->child->get_ZS()->IS, aic->fsm->states)
				   << "},{"
				   << get_subset_string(y_Dbts->child->CD, aic->fsm->events)
				   << "}}\n";
		}
		os << "Z-States:\n";
		for (Z_DBTS* z_Dbts : ZSL) {
			os << "\tZ_DBTS State: {{"
			   << get_subset_string(z_Dbts->get_ZS()->IS, aic->fsm->states) 
			   << "},{"
			   << get_subset_string(z_Dbts->CD, aic->fsm->events)
			   << "}}\n"
			   << (z_Dbts->children.empty() ? "" : "\tTransitions to:\n");
			for (auto& pair : z_Dbts->children)
				os << "\t\tY_DBTS State {"
				   << get_subset_string(pair.second->get_YS()->IS, aic->fsm->states)
				   << "} via Event "
				   << aic->fsm->events.get_key(pair.first) << "\n";
		}
	}
	os << "******************************************************************************\n"
	   << "********************Determined Bipartite Transition System********************\n"
	   << "******************************************************************************\n";
}
