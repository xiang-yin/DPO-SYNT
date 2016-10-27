#include <algorithm>
#include <numeric>
#include <cmath>
#include <stack>
#include "../include/NBAIC.h"
#include "../include/Utilities.h"
using namespace std;

CONTROL_DECISION convert_to_all_events(const CONTROL_DECISION& CD,
									   const vector<int>& max_CD,
									   const int nevents);
bool all_deleted(unordered_map<CONTROL_DECISION, ZS*,
				 			   hash<CONTROL_DECISION>>& transition);
bool exists_deleted(unordered_map<EVENT, YS*>& transition);
template <typename BOOL_CONTAINER>
void record_path(BOOL_CONTAINER& bc, Node<STATE>* current);
bool IS_match(const NBAIC_State* nbs, const INFO_STATE& IS, bool& match);
template <typename NBAIC_MAP>
void push_children(queue<Node<NBAIC_State*>*>& BFS, 
				   Node<NBAIC_State*>* parent, NBAIC_MAP& nbaic_map);
template <typename State_List>
void mark_deleted_helper(vector<bool>& accessible,
						 int& index, STATE s, State_List& sl);
bool currently_unobservable(FSM* fsm, const SENSING_DECISION& current_SD,
							const EVENT e);
bool currently_unobservable(FSM* fsm, const SENSING_DECISION& current_SD,
							const vector<int>& max_SD, const EVENT e);
void remove_transition(YS* ys, const SENSING_DECISION* SD);
string get_title(Mode mode, bool print_BDO, bool is_end);

struct Is_Child {
	Is_Child(NBAIC_State* parent_) : parent(parent_) {}
	bool operator()(NBAIC_State* nbs) { return nbs == parent; }
	NBAIC_State* parent;
};


///////////////////////////////////////////////////////////////////////////////


NBAIC::NBAIC(FSM* fsm_, IS_Property* isp_,
			 ostream& os_, Mode mode_   /*=BSCOPNBMAX*/ )
	: fsm(fsm_), ISP(isp_), ics(fsm, os_), os(os_), mode(mode_) {
	if (fsm->is_invalid(mode)) return;
	NBAIC_State::nstates = fsm->nstates;
	NBAIC_State::nevents = fsm->nevents;
	/* Create initial Y-State */
	YS* y0 = new YS();
	y0->IS[0] = true;
	YSL.push_back(y0);

	if (mode == BSCOPNBMAX) {
		DoDFS_BSCOPNBMAX(y0);
		bool root_is_coaccessible = true;
		do {
			prune();
			delete_inaccessible();
		} while (ics.exists_livelock(root_is_coaccessible));
		if (!root_is_coaccessible) delete_states();
	}
	else if (mode == MPO) {
		DoDFS_MPO(y0);
		prune();
		delete_inaccessible();
	}
	else if (mode == MPRCP){
		DoDFS_MPRCP(y0);
		prune();
		delete_inaccessible();
	}
}

NBAIC::~NBAIC() {
	delete_states();	
}


///////////////////////////////////////////////////////////////////////////////


void NBAIC::DoDFS_BSCOPNBMAX(YS* ys) {
	vector<int> max_CD = get_max_CD(ys);
	/* For each member of power set of valid, controllable events */
	for (unsigned i = 0; i < pow(2, max_CD.size()); ++i) {
		CONTROL_DECISION current_CD(max_CD.size(), false);
		unsigned event_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is 1 */
			if (event_pset & (unsigned) 0x1) current_CD[j] = true;
			/* Shift all bits right */
			event_pset >>= 1;
			/* No more one bits */
			if (event_pset == 0) break;
		}
		CONTROL_DECISION used_events(max_CD.size(), false);
		vector<Transition> UR_transitions;
		INFO_STATE IS = unobservable_reach(ys, current_CD, used_events,
										   max_CD, UR_transitions);

		/* Unused event in current_CD || doesn't satisfy IS property || deadlocked */
		if (redundant(current_CD, used_events) || !(*ISP)(IS)
			|| is_deadlocked(IS, current_CD, max_CD)) continue;

		bool zs_in_ZSL = false;
		current_CD = convert_to_all_events(current_CD, max_CD, fsm->nevents);
		ZS* zs = get_ZS(IS, current_CD, zs_in_ZSL);
		/* Link ys to child zs */
		ys->transition[current_CD] = zs;
		/* Link zs to parent ys */
		zs->reverse[current_CD].push_back(ys);

		ics.push(ys, zs, current_CD);
		ics.push(zs, (NBAIC_State*&)zs, UR_transitions);

		if (!zs_in_ZSL) {
			ZSL.push_back(zs);
			/* For all observable, controllable events in control decision */
			for (EVENT e = 0; e < current_CD.size(); ++e) {
				vector<Transition> OR_transitions;
				if (!current_CD[e] || !fsm->observable[e]
					|| !observable_reach(IS, zs, e, OR_transitions)) continue;

				YS* next_ys = nullptr;
				if (!ys_in_YSL(next_ys, zs, IS, e, OR_transitions)) {
					YSL.push_back(next_ys);
					DoDFS_BSCOPNBMAX(next_ys);
				}
			}
			/* For all observable, uncontrollable events */
			for (EVENT e = 0; e < fsm->uo.size(); ++e) {
				vector<Transition> OR_transitions;
				if (!observable_reach(IS, zs, fsm->uo[e], OR_transitions)) continue;

				YS* next_ys = nullptr;
				if (!ys_in_YSL(next_ys, zs, IS, fsm->uo[e], OR_transitions)) {
					YSL.push_back(next_ys);
					DoDFS_BSCOPNBMAX(next_ys);
				}
			}
		}
	}
}

void NBAIC::DoDFS_MPRCP(YS* ys) {
	vector<int> max_CD = get_max_CD(ys);
	/* For each member of power set of valid, controllable events */
	for (unsigned i = 0; i < pow(2, max_CD.size()); ++i) {
		CONTROL_DECISION current_CD(max_CD.size(), false);
		unsigned event_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is 1 */
			if (event_pset & (unsigned) 0x1) current_CD[j] = true;
			/* Shift all bits right */
			event_pset >>= 1;
			/* No more one bits */
			if (event_pset == 0) break;
		}
		CONTROL_DECISION used_events(max_CD.size(), false);
		vector<Transition> UR_transitions;
		INFO_STATE IS = unobservable_reach(ys, current_CD, used_events,
										   max_CD, UR_transitions);

		/* Unused event in current_CD || doesn't satisfy IS property*/
		if (redundant(current_CD, used_events) || !(*ISP)(IS)) continue;

		bool zs_in_ZSL = false;
		current_CD = convert_to_all_events(current_CD, max_CD, fsm->nevents);
		ZS* zs = get_ZS(IS, current_CD, zs_in_ZSL);
		/* Link ys to child zs */
		ys->transition[current_CD] = zs;
		/* Link zs to parent ys */
		zs->reverse[current_CD].push_back(ys);
		if (!zs_in_ZSL) {
			ZSL.push_back(zs);
			/* For all observable, controllable events in control decision */
			for (EVENT e = 0; e < current_CD.size(); ++e) {
				vector<Transition> OR_transitions;
				if (!current_CD[e] || !fsm->observable[e]
					|| !observable_reach(IS, zs, e, OR_transitions)) continue;

				YS* next_ys = nullptr;
				if (!ys_in_YSL(next_ys, zs, IS, e, OR_transitions)) {
					YSL.push_back(next_ys);
					DoDFS_MPRCP(next_ys);
				}
			}
			/* For all observable, uncontrollable events */
			for (EVENT e = 0; e < fsm->uo.size(); ++e) {
				vector<Transition> OR_transitions;
				if (!observable_reach(IS, zs, fsm->uo[e], OR_transitions)) continue;

				YS* next_ys = nullptr;
				if (!ys_in_YSL(next_ys, zs, IS, fsm->uo[e], OR_transitions)) {
					YSL.push_back(next_ys);
					DoDFS_MPRCP(next_ys);
				}
			}
		}
	}
}

void NBAIC::DoDFS_MPO(YS* ys) {
	// print();
	vector<int> max_SD = get_max_CD(ys);
	/* For each member of power set of valid, monitorable events */
	for (unsigned i = 0; i < pow(2, max_SD.size()); ++i) {
		SENSING_DECISION current_SD(fsm->nevents, false);
		unsigned event_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is 1 */
			if (event_pset & (unsigned) 0x1) current_SD[max_SD[j]] = true;
			/* Shift all bits right */
			event_pset >>= 1;
			/* No more one bits */
			if (event_pset == 0) break;
		}
		SENSING_DECISION used_events(fsm->nevents, false);
		vector<Transition> UR_transitions;
		INFO_STATE IS = unobservable_reach(ys, current_SD, used_events,
										   max_SD, UR_transitions);

		/* Unused event in current_SD || doesn't satisfy IS property || deadlocked */
		if (redundant(current_SD, used_events) || !(*ISP)(IS)
			/*|| is_deadlocked(IS, current_SD, max_SD)*/) continue;

		bool zs_in_ZSL = false;
		ZS* zs = get_ZS(IS, current_SD, zs_in_ZSL);
		/* Link ys to child zs */
		ys->transition[current_SD] = zs;
		/* Link zs to parent ys */
		zs->reverse[current_SD].push_back(ys);

		if (!zs_in_ZSL) {
			ZSL.push_back(zs);
			/* For all observable, non-redundant events */
			for (EVENT e = 0; e < fsm->nevents; ++e) {
				vector<Transition> OR_transitions;
				if (currently_unobservable(fsm, current_SD, e)
					|| !observable_reach(IS, zs, e, OR_transitions)) continue;

				YS* next_ys = nullptr;
				if (!ys_in_YSL(next_ys, zs, IS, e, OR_transitions)) {
					YSL.push_back(next_ys);
					DoDFS_MPO(next_ys);
				}
			}
		}
	}
}

void NBAIC::reduce_MPO(bool generate_maximal) {
	unordered_map<NBAIC_State*, int> YS_index = create_NBAIC_index(true, false);
	vector<bool> visited(YS_index.size());
	queue<YS*> BFS;
	BFS.push(YSL[0]);
	while (!BFS.empty()) {
		YS* ys = BFS.front();
		BFS.pop();

		/* Only visit each Y-state once */
		if (visited[YS_index[ys]]) continue;
		visited[YS_index[ys]] = true;

		const SENSING_DECISION* greedy_SD = nullptr;
		/* For each transition defined at the Y-state */
		auto iter = ys->transition.begin();
		while (iter != ys->transition.end()) {
			auto current = iter++;
			/* Pick first state--if transition is singleton then trivially optimal */
			if (!greedy_SD) {
				greedy_SD = &current->first;
				continue;
			}
			/* Transition is more optimal based on max/min mode */
			if ((generate_maximal && is_subset(*greedy_SD, current->first))
				|| (!generate_maximal && is_subset(current->first, *greedy_SD))) {
				remove_transition(ys, greedy_SD);
				greedy_SD = &current->first;
			}
			/* Transition is not more optimal and should be deleted */
			else {
				remove_transition(ys, &current->first);
			}
		}

		/* Add all children of Z-state to the queue */
		ZS* zs = ys->transition[*greedy_SD];
		for (auto& pair : zs->transition)
			BFS.push(pair.second);
	}
	/* Clean up the transitions we deleted */
	delete_inaccessible();
}

SENSING_DECISION NBAIC::flag_observable(const SENSING_DECISION& SD) {
	SENSING_DECISION result = SD;
	for (EVENT e : fsm->uo)
		result[e] = true;
	return result;
}

void NBAIC::prune() {
	if (is_empty()) {
		delete_states();
		return;
	}
	bool pruning = true;
	/* First iteration || the previous iteration deleted some states */
	while (pruning) {
		pruning = false;
		for (YS* ys : YSL) {
			if (ys->deleted) continue;
			/* Y-State has no successor and should be deleted */
			if (ys->transition.empty() || all_deleted(ys->transition))
				ys->deleted = pruning = true;
		}
		for (ZS* zs : ZSL) {
			if (zs->deleted || zs->transition.empty()) continue;
			/* Z-State has a deleted successor and should be deleted */
			if (exists_deleted(zs->transition))
				zs->deleted = pruning = true;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////


INFO_STATE NBAIC::unobservable_reach(const YS* ys, const CONTROL_DECISION& CD,
									 CONTROL_DECISION& used_events,
									 const vector<int>& max_CD,
									 vector<Transition>& UR_transitions) {
	INFO_STATE result(fsm->nstates, false);
	queue<STATE> BFS;

	/* Fill BFS with states in current IS */
	for (STATE s = 0; s < ys->IS.size(); ++s)
		if (ys->IS[s]) BFS.push(s);

	while (!BFS.empty()) {
		STATE current = BFS.front();
		BFS.pop();
		result[current] = true;

		if (mode != MPO) {
			/* For all controllable events in control decision */
			for (EVENT e = 0; e < max_CD.size(); ++e) {
				/* Event is in CD and is valid for current state */
				if (CD[e] && fsm->transitions[current].find(max_CD[e])
						  != fsm->transitions[current].end()) {
					used_events[e] = true;
					/* Event is unobservable, so next state must be
					in unobservable reach */
					if (!fsm->observable[max_CD[e]])
						find_next(BFS, result, current, max_CD[e], UR_transitions);
				}
			}
			/* For all uncontrollable, unobservable events */
			for (EVENT e = 0; e < fsm->uu.size(); ++e) {
				/* Event is valid for current state */
				if (fsm->transitions[current].find(fsm->uu[e])
					!= fsm->transitions[current].end()) {
					find_next(BFS, result, current, fsm->uu[e], UR_transitions);
				}
			}
		}
		else if (mode == MPO) {
			/* For all unobservable events in reach */
			for (auto& pair : fsm->transitions[current])
				/* Event is unobservable and child
				state hasn't been visited */
				if (currently_unobservable(fsm, CD, pair.first)
					&& !result[pair.second]) BFS.push(pair.second);
				else if (CD[pair.first]) used_events[pair.first] = true;
		}
	}
	return result;
}

bool NBAIC::observable_reach(INFO_STATE& IS, const ZS* zs, const EVENT e,
						     vector<Transition>& OR_transitions) {
	fill(IS.begin(), IS.end(), false);
	bool has_OR = false; /* has Observable Reach */
	for (STATE s = 0; s < zs->IS.size(); ++s) {
		/* Event is valid for the state in Information State of Z-State */
 		if (zs->IS[s] && fsm->transitions[s].find(e) !=
 						 fsm->transitions[s].end()) {
 			/* Add the state that results from the transition */
 			STATE next = fsm->transitions[s][e];
			/* Add ICS transition information */
 			if (mode != MPO) {
	 			Transition transition(s, e, next);
	 			OR_transitions.push_back(transition);
 			}
			IS[next] = true;
			has_OR = true;
		}
	}
	return has_OR;
}

void NBAIC::find_next(queue<STATE>& BFS, const INFO_STATE& result,
					  const STATE current, const EVENT e,
					  vector<Transition>& UR_transitions) {
	STATE next = fsm->transitions[current][e];
	/* Add ICS transition information */
	if (mode == BSCOPNBMAX) {
		Transition transition(current, e, next);
		UR_transitions.push_back(transition);
	}
	/* Haven't visited next state */
	if (!result[next]) BFS.push(next);
}


///////////////////////////////////////////////////////////////////////////////


/* Finds the maximum possible control decision with no
redundancy for the given Y-State in polynomial time */
vector<int> NBAIC::get_max_CD(const YS* ys) {
	CONTROL_DECISION CD(fsm->nevents);
	INFO_STATE visited(ys->IS.size(), false);
	queue<STATE> BFS; /* Breadth First Search */

	/* Mark every state in current Y-State as visited */
	for (STATE s = 0; s < ys->IS.size(); ++s)
		if (ys->IS[s]) BFS.push(s);

	while (!BFS.empty()) {
		STATE current = BFS.front();
		BFS.pop();

		/* Only visit each state at most once */
		if (visited[current]) continue;
		visited[current] = true;

		/* Mark each valid event for the state */
		for (EVENT e = 0; e < CD.size(); ++e) {
			if (fsm->transitions[current].find(e)
				!= fsm->transitions[current].end()) {
				if ((mode != MPO && fsm->controllable[e]) ||
					(mode == MPO && fsm->monitorable[e])) CD[e] = true;
				/* Get the next state only if its
				within the unobservable reach*/
				if (!fsm->observable[e]) {
					STATE next = fsm->transitions[current][e];
					if (!visited[next]) BFS.push(next);
				}
			}
		}
	}
	/* Prepare set of integers representing valid
	events for use in power set */
	vector<int> result;
	for (EVENT e = 0; e < CD.size(); ++e)
		if (CD[e]) result.push_back(e);

	return result;
}

ZS* NBAIC::get_ZS(const INFO_STATE& IS, const CONTROL_DECISION& CD, bool& zs_in_ZSL) {
	/* For all Y-States that can transition via CD */
	for (YS* ys : YSL) {
		if (ys->transition.find(CD) == ys->transition.end()) continue;
		ZS* zs = ys->transition[CD];
		/* Identical Z-State already exists in NBAIC */
		if (IS_match(zs, IS, zs_in_ZSL)) return zs;
	}
	/* No identical Z-State exists in NBAIC--create one */
	return new ZS(IS, CD);
}

YS* NBAIC::get_YS(const INFO_STATE& IS, bool& ys_in_YSL) {
	for (YS* ys : YSL)
		/* Identical Y-state already exists in NBAIC */
		if (IS_match(ys, IS, ys_in_YSL)) return ys;
	/* No identical Y-State exists in NBAIC--create one */
	return new YS(IS);
}

bool NBAIC::ys_in_YSL(YS*& ys, ZS* zs, const INFO_STATE& IS, const EVENT e,
					  vector<Transition>& OR_transitions) {
	bool result = false;
	ys = get_YS(IS, result);
	/* Link zs to child ys */
	zs->transition[e] = ys;
	/* Link yz to parent zs */
	ys->reverse[e].push_back(zs);
	if (mode == BSCOPNBMAX) ics.push(zs, ys, OR_transitions);
	return result;
}

CONTROL_DECISION NBAIC::get_CD(ZS* zs) {
	for (YS* ys : YSL)
		for (auto& pair : ys->transition)
			if (pair.second == zs) return pair.first;
}

///////////////////////////////////////////////////////////////////////////////


/* Clean up memory and shorten state lists down to only what we need */
template <typename State_List>
void NBAIC::delete_NBAIC_States(State_List& sl) {
	unsigned num_deleted = 0;
	/* For all states in the state list marked as deleted */
	for (STATE s = 0; s < sl.size(); ++s) {
		if (sl[s]->deleted) {
			if (mode == BSCOPNBMAX) delete_ICS_State(sl[s]);
			/* Delete all links from parent to current state */
			for (auto& pair : sl[s]->reverse)
				for (auto* parent : pair.second)
					if (parent->transition.find(pair.first) !=
						parent->transition.end())
						parent->transition.erase(pair.first);
			/* Delete all links from child to current state */
			for (auto& pair : sl[s]->transition) {
				auto& vec = pair.second->reverse[pair.first];
				vec.erase(remove_if(vec.begin(), vec.end(), Is_Child(sl[s])),
						  vec.end());
			}
			/* Delete current state */
			delete sl[s];
			sl[s] = nullptr;
			++num_deleted;
		}
		/* Move remaining states over so deletion of any number
		of states can occur in O(n) without leaving any gaps in the vector */
		else if (num_deleted > 0) {
			sl[s - num_deleted] = sl[s];
		}
	}
	sl.resize(sl.size() - num_deleted);
}

void NBAIC::delete_ICS_State(NBAIC_State* nbs) {
	/* For all ICS_States corresponding to the NBAIC_State to be deleted */
	for (STATE s = 0; s < nbs->IS.size(); ++s)
		if (nbs->IS[s]) ics.pop(nbs, s);
}

void NBAIC::delete_inaccessible() {
	if (is_empty()) return;
	unordered_map<NBAIC_State*, int> NBAIC_index = create_NBAIC_index(true, true);
	vector<bool> accessible(NBAIC_index.size(), false);

	queue<Node<NBAIC_State*>*> BFS; /* Breadth First Search */
	Node<NBAIC_State*>* root = new Node<NBAIC_State*>(YSL[0]);
	BFS.push(root);
	while(!BFS.empty()) {
		Node<NBAIC_State*>* current = BFS.front();
		BFS.pop();

		int index = NBAIC_index[current->val];
		/* Already visited--no need to visit again */
		if (accessible[index]) continue;
		accessible[index] = true;
		/* Add all accessible children */
		if (current->val->is_YS) {
			YS* ys = dynamic_cast<YS*>(current->val);
			push_children(BFS, current, ys->transition);
		}
		else {
			ZS* zs = dynamic_cast<ZS*>(current->val);
			push_children(BFS, current, zs->transition);
		}
	}
	reset_tree(root);
	mark_deleted(accessible);
	delete_NBAIC_States(YSL);
	delete_NBAIC_States(ZSL);
}

void NBAIC::mark_deleted(vector<bool>& accessible) {
	int index = 0;
	/* Delete all inaccessible Y-States */
	while (index < YSL.size())
		mark_deleted_helper(accessible, index, index, YSL);
	/* Delete all inaccessible Z-States
	Note: accessible is partitioned so that the last ZSL.size()
	states represent the Z-States of the NBAIC */
	while (index < accessible.size())
		mark_deleted_helper(accessible, index, index - YSL.size(), ZSL);
}

void NBAIC::delete_states() {
	for (YS* ys : YSL) delete ys;
	for (ZS* zs : ZSL) delete zs;
	YSL.clear();
	ZSL.clear();
}


///////////////////////////////////////////////////////////////////////////////


/* If there is a controllable event in the CD that we did not use
   then the control decision is redundant */
bool NBAIC::redundant(const CONTROL_DECISION& CD,
					  const CONTROL_DECISION& used) const {
	for (EVENT e = 0; e < CD.size(); ++e)
		/* Event found in CD that was never used--CD is redundant */
		if (CD[e] && !used[e]) return true;
	/* No redundant event found */
	return false;
}

/* Returns true if there exists a state in IS that cannot reach
   either a marked state or an observable state */
bool NBAIC::is_deadlocked(const INFO_STATE& IS, const CONTROL_DECISION& CD,
						  const vector<int>& max_CD) {
	INFO_STATE non_deadlocked(IS.size(), false);
	for (STATE s = 0; s < IS.size(); ++s) {
		if (!IS[s] || non_deadlocked[s]) continue;

		INFO_STATE visited(IS.size(), false);
		queue<Node<STATE>*> BFS;
		Node<STATE>* root = new Node<STATE>(s);
		BFS.push(root);
		while (!BFS.empty()) {
			Node<STATE>* current = BFS.front();
			BFS.pop();

			/* State initially marked or already found to be deadlock
			free in the path of a previously checked state */
			if (non_deadlocked[current->val] || fsm->marked[current->val]) {
				record_path(non_deadlocked, current);
				break;
			}

			if (visited[current->val]) continue;
			visited[current->val] = true;
			if (mode == BSCOPNBMAX)
				check_BSCOPNBMAX_deadlock(max_CD, BFS, current,
										  non_deadlocked, CD);
			else if (mode == MPO) 
				check_MPO_deadlock(max_CD, BFS, current,
								  non_deadlocked, CD);
			if (non_deadlocked[s]) break;
		}
		reset_tree(root);
		/* Breadth first search couldn't find a path to an observable event or marked state */
		if (!non_deadlocked[s]) return true;
	}
	return false;
}

void NBAIC::check_BSCOPNBMAX_deadlock(const vector<int>& max_CD,
									  queue<Node<STATE>*>& BFS,
									  Node<STATE>* current,
									  INFO_STATE& non_deadlocked,
									  const CONTROL_DECISION& CD) {
	/* For all controllable events in CD defined at current state */
	for (EVENT e = 0; e < max_CD.size(); ++e) {
		if (CD[e] && fsm->transitions[current->val].find(max_CD[e])
			!= fsm->transitions[current->val].end())
			check_state_deadlock(BFS, current, max_CD[e], non_deadlocked, CD);
	}
	/* For all uncontrollable, unobservable events in CD defined at current state */
	for (EVENT e = 0; e < fsm->uu.size(); ++e) {
		if (fsm->transitions[current->val].find(fsm->uu[e])
			!= fsm->transitions[current->val].end())
			check_state_deadlock(BFS, current, fsm->uu[e], non_deadlocked, CD);
	}
	/* For all uncontrollable, observable events in CD defined at current state */
	for (EVENT e = 0; e < fsm->uo.size(); ++e) {
		if (fsm->transitions[current->val].find(fsm->uo[e])
			!= fsm->transitions[current->val].end())
			check_state_deadlock(BFS, current, fsm->uo[e], non_deadlocked, CD);
	}
}

void NBAIC::check_MPO_deadlock(const vector<int>& max_SD,
							   queue<Node<STATE>*>& BFS,
							   Node<STATE>* current,
							   INFO_STATE& non_deadlocked,
							   const SENSING_DECISION& current_SD) {
	for (EVENT e = 0; e < fsm->nevents; ++e) {
		if (fsm->transitions[current->val].find(e)
			!= fsm->transitions[current->val].end())
			check_state_deadlock(BFS, current, e, non_deadlocked, current_SD);
	}
}

void NBAIC::check_state_deadlock(queue<Node<STATE>*>& BFS, Node<STATE>* current,
								 const EVENT e, INFO_STATE& non_deadlocked,
								 const SENSING_DECISION& current_SD) {
	/* Observable event defined for state within UR--state is not deadlocked */
	if (fsm->observable[e] || (mode == MPO &&
							   !currently_unobservable(fsm, current_SD, e)))
		record_path(non_deadlocked, current);
	else {
		Node<STATE>* next = new Node<STATE>(fsm->transitions[current->val][e]);
		next->prev = current;
		current->next.push_back(next);
		/* Marked state exists in UR--state is not deadlocked */
		if (fsm->marked[next->val]) record_path(non_deadlocked, next);
		/* Unobservable state exists in UR--keep going */
		else BFS.push(next);
	}
}


///////////////////////////////////////////////////////////////////////////////


/* Map each NBAIC_State to a unique int */
unordered_map<NBAIC_State*, int> NBAIC::create_NBAIC_index(bool Y, bool Z) const {
	unordered_map<NBAIC_State*, int> NBAIC_index;
	NBAIC_index.reserve(YSL.size() + ZSL.size());
	int count = 0;
	if (Y)
		for (YS* ys : YSL) NBAIC_index[ys] = count++;
	if (Z)
		for (ZS* zs : ZSL) NBAIC_index[zs] = count++;
	return NBAIC_index;
}


///////////////////////////////////////////////////////////////////////////////


void NBAIC::print_sets(const INFO_STATE& IS, const CONTROL_DECISION& CD) {
	os << "Information State: {" << get_subset_string(IS, fsm->states) << "}\n";
	os << "Control Decision: {" << get_subset_string(CD, fsm->events) << "}\n";
}

void NBAIC::print(bool print_BDO /* = false */) {
	os << "*******************************************************************\n"
	   << "*************" << get_title(mode, print_BDO, false)
	   << "*****************\n"
	   << "*******************************************************************\n";
	os << "Y-States:\n";
	for (YS* ys : YSL) {
		os << "\tInformation State: {"
		   << get_subset_string(ys->IS, fsm->states) << "}\n"
		   << (ys->transition.empty() ? "" : "\tTransitions to:\n");
		for (auto& pair : ys->transition)
			os << "\t\tZ-State with Information State {{"
			   << get_subset_string(pair.second->IS, fsm->states) <<"},{"
			   << get_subset_string((mode == MPO ? flag_observable(pair.first)
			   									 : pair.first), fsm->events)
			   << "}} via "
			   << (mode != MPO ? "Control Decision" : "Sensing Decision")
			   << " {"
			   << get_subset_string((mode == MPO ? flag_observable(pair.first)
			   									 : pair.first), fsm->events)
			   << "}\n";
	}
	os << "Z-States:\n";
	for (ZS* zs : ZSL) {
		os << "\tInformation State: {{"
		   << get_subset_string(zs->IS, fsm->states) <<"},{"
		   << get_subset_string((mode == MPO ? flag_observable(zs->CD)
			   									 : zs->CD), fsm->events)
		   << "}}\n"
		   << (zs->transition.empty() ? "" : "\tTransitions to:\n");
		for (auto& pair : zs->transition)
			os << "\t\tY-State with Information State {"
			   << get_subset_string(pair.second->IS, fsm->states)
			   << "} via Event "
			   << fsm->events.get_key(pair.first) << "\n";
	}
	os << "*******************************************************************\n"
	   << "*************" << get_title(mode, print_BDO, true)
	   << "*************\n"
	   << "*******************************************************************\n";
}

void NBAIC::print_fsm(const char* const filename) {
	ofstream file_os(filename);
	file_os << YSL.size() + ZSL.size() << "\r\n\r\n";
	for (YS* ys : YSL) {
		file_os << '{' << get_subset_string(ys->IS, fsm->states) << "}	0	"
				<< ys->transition.size() << "\r\n";
		for (auto& pair : ys->transition)
			file_os << '{' << get_subset_string((mode == MPO ? flag_observable(pair.first)
			   												 : pair.first), fsm->events)
					<< "}	{{" << get_subset_string(pair.second->IS, fsm->states)
					<< "},{" << get_subset_string((mode == MPO ? flag_observable(pair.first)
			   									 			   : pair.first), fsm->events)
					<< "}}	c	o\r\n";
		file_os << "\r\n";
	}
	for (ZS* zs : ZSL) {
		CONTROL_DECISION CD = get_CD(zs);
		file_os << "{{" << get_subset_string(zs->IS, fsm->states) << "},{"
				<< get_subset_string((mode == MPO ? flag_observable(CD)
			   									  : CD), fsm->events) << "}}	1	"
				<< zs->transition.size() << "\r\n";
		for (auto& pair : zs->transition)
			file_os << fsm->events.get_key(pair.first) << "	{"
					<< get_subset_string(pair.second->IS, fsm->states) << "}	"
					<< print_event_parameters(pair.first) << "\r\n";
		file_os << "\r\n";
	}
	file_os<<endl;
	file_os.close();
}

string NBAIC::print_event_parameters(EVENT e) {
	string result; 
	if (mode != MPO)
		result += (fsm->controllable[e] ? "c	" : "uc	");
	/* DESUMA does not support monitorable symbols,
	so we repurpose controllability for MPO */
	else if (mode == MPO)
		result += (fsm->monitorable[e] ? "c	" : "uc	");
	result += (fsm->observable[e] ? "o" : "uo");
	return result;
}


///////////////////////////////////////////////////////////////////////////////


void NBAIC::run_tests() {
	/* Generate and run tests for each member of the power set of IS */
	for (unsigned i = 0; i < pow(2, fsm->nstates); ++i) {
		INFO_STATE current_IS(fsm->nstates, false);
		unsigned state_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is a 1 */
			if (state_pset & (unsigned) 0x1) current_IS[j] = true;
			/* Shift all bits right */
			state_pset >>= 1;
			if (state_pset == 0) break;
		}
		os << "Information State: {"
		   << get_subset_string(current_IS, fsm->states) << "}\n";

		deadlock_test(current_IS);
		// opacity_test(current_IS);
		// safety_test(current_IS);
	}
}

void NBAIC::deadlock_test(const INFO_STATE& IS) {
	for (unsigned k = 0; k < pow(2, fsm->events.size()); ++k) {
		CONTROL_DECISION current_CD(fsm->events.size(), false);
		unsigned event_pset = k;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is 1 */
			if (event_pset & (unsigned) 0x1) current_CD[j] = true;
			/* Shift all bits right */
			event_pset >>= 1;
			if (event_pset == 0) break;
		}
		vector<int> max_CD(fsm->nevents);
		iota(max_CD.begin(), max_CD.end(), 0);
		os << "Control Decision: {"
		   << get_subset_string(current_CD, fsm->events) << "}\n"
		   << "Deadlock: "
		   << (is_deadlocked(IS, current_CD, max_CD) ? "yes" : "no") << "\n\n";
	}
}

void NBAIC::opacity_test(const INFO_STATE& IS) {
	for (unsigned i = 0; i < pow(2, fsm->states.size()); ++i) {
		INFO_STATE secret_states(fsm->states.size());
		unsigned state_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is a 1 */
			if (state_pset & (unsigned) 0x1) secret_states[j] = true;
			/* Shift all bits right */
			state_pset >>= 1;
			if (state_pset == 0) break;
		}
		Opacity* opacity = new Opacity(secret_states);
		os << "Reveals {" << get_subset_string(secret_states, fsm->states)
		   << "}: " << ((*opacity)(IS) ? "no" : "yes") << '\n';
	}
}

void NBAIC::safety_test(const INFO_STATE& IS) {
	for (unsigned i = 0; i < pow(2, fsm->states.size()); ++i) {
		INFO_STATE unsafe_states(fsm->states.size());
		unsigned state_pset = i;
		for (unsigned j = 0; j < sizeof(unsigned) * 8; ++j) {
			/* First bit is a 1 */
			if (state_pset & (unsigned) 0x1) unsafe_states[j] = true;
			/* Shift all bits right */
			state_pset >>= 1;
			if (state_pset == 0) break;
		}
		Safety* safety = new Safety(unsafe_states);
		os << "Safe {" << get_subset_string(unsafe_states, fsm->states)
		   << "}: " << ((*safety)(IS) ? "yes" : "no") << '\n';
	}
}


///////////////////////////////////////////////////////////////////////////////


/* Converts a control decision based on a local, maximum CD to
an equivalent global CD */
CONTROL_DECISION convert_to_all_events(const CONTROL_DECISION& CD,
									   const vector<int>& max_CD,
									   const int nevents) {
	CONTROL_DECISION result(nevents);
	for (EVENT e = 0; e < CD.size(); ++e)
		if (CD[e]) result[max_CD[e]] = true;
	return result;
}

template <typename BOOL_CONTAINER>
void record_path(BOOL_CONTAINER& bc, Node<STATE>* current) {
	bc[current->val] = true;
	if (!current->prev) return;
	else record_path(bc, current->prev);
}

bool all_deleted(unordered_map<CONTROL_DECISION, ZS*,
				 			   hash<CONTROL_DECISION>>& transition) {
	auto iter = transition.begin();
	while (iter != transition.end()) {
		auto current = iter++;
		/* Y-State preceeds a deleted Z-State */
		if (!(*current).second->deleted) return false;
	}
	return true;
}

bool exists_deleted(unordered_map<EVENT, YS*>& transition) {
	auto iter = transition.begin();
	while (iter != transition.end()) {
		auto current = iter++;
		/* Z-State preceeds a deleted Y-State and should be erased */
		if ((*current).second->deleted)	return true;
	}
	/* Z-State currently does not preceed a deleted Y-State */
	return false;
}

bool IS_match(const NBAIC_State* nbs, const INFO_STATE& IS, bool& match) {
	if (nbs->IS == IS) match = true;
	return match;
}

template <typename NBAIC_MAP>
void push_children(queue<Node<NBAIC_State*>*>& BFS,
				   Node<NBAIC_State*>* parent, NBAIC_MAP& nbaic_map) {
	/* For each child state that has not been deleted */
	for (auto& pair : nbaic_map)
		if (!pair.second->deleted) {
			/* Add child to the queue */
			Node<NBAIC_State*>* child = new Node<NBAIC_State*>(pair.second);
			child->prev = parent;
			parent->next.push_back(child);
			BFS.push(child);
		}
}

template <typename State_List>
void mark_deleted_helper(vector<bool>& accessible, int& index,
						 STATE s, State_List& sl) {
	/* Delete state sl[s] if it is inaccessble */
	if (!accessible[index]) sl[s]->deleted = true;
	++index;
}


bool currently_unobservable(FSM* fsm, const SENSING_DECISION& current_SD,
							const EVENT e) {
	return (fsm->monitorable[e] && !current_SD[e])
		|| !(fsm->monitorable[e] || fsm->observable[e]);
}

bool currently_unobservable(FSM* fsm, const SENSING_DECISION& current_SD,
							const vector<int>& max_SD, const EVENT e) {
	return (fsm->monitorable[max_SD[e]] && !current_SD[e])
		|| !(fsm->monitorable[max_SD[e]] || fsm->observable[max_SD[e]]);
}

void remove_transition(YS* ys, const SENSING_DECISION* SD) {
	ys->transition[*SD]->reverse.erase(*SD);
	ys->transition.erase(*SD);
}

string get_title(Mode mode, bool print_BDO, bool is_end) {
	string ending;
	if (is_end) ending = "End ";
	if (mode == BSCOPNBMAX) return ending + "Non-Blocking All Inclusive Controller";
	else if (mode == MPRCP) return "********" + ending +"All Inclusive Controller*****";
	else if (print_BDO) return "******" + ending + "Bipartite Dynamic Observer*****";
	else return "*******" + ending + "Most Permissive Observer******";
}


///////////////////////////////////////////////////////////////////////////////
