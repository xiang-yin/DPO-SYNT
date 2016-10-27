#include <algorithm>
#include "../include/ICS.h"
#include "../include/UBTS.h"
#include "../include/Utilities.h"
using namespace std;

template <typename BOOL_CONTAINER>
static void record_path(BOOL_CONTAINER& bc, Node<ICS_STATE*>* current,
				 		unordered_map<ICS_STATE*, int>& ics_index);
template <typename ICS_MAP>
void find_next_state(stack<Node<ICS_STATE*>*>& DFS, Node<ICS_STATE*>* current,
					 ICS_MAP& ics_map);
void push_child_to_stack(stack<Node<ICS_STATE*>*>& DFS,
						 Node<ICS_STATE*>* parent,
						 Node<ICS_STATE*>* child);
void backtrack(Node<ICS_STATE*>* current);
Node<ICS_STATE*>* make_child(Node<ICS_STATE*>* current, ICS_STATE* child);


///////////////////////////////////////////////////////////////////////////////


ICS::ICS(UBTS& ubts, FSM* fsm_, ostream& os_) : fsm(fsm_), os(os_) {
	ubts.mark_ambiguities();
	/* Add transitions from Y_UBTS states to Z_UBTS states*/
	for (Y_UBTS* y_ubts : ubts.YSL)
		if (y_ubts->child)
			push(y_ubts->get_YS(), y_ubts->child->get_ZS(),
				 y_ubts->child->CD, y_ubts->num_predecessors,
				 y_ubts->child->num_predecessors,
				 y_ubts->disambiguation_state);
		else {
			YS* ys = y_ubts->get_YS();
			for (STATE s = 0; s < ys->IS.size(); ++s) {
				if (!ys->IS[s]) continue;
				ICS_STATE* y_ics = get_ICS_STATE(y_ubts->nbs, s,
												 y_ubts->num_predecessors);
				ICS_STATE* y_ics_0 = get_ICS_STATE(y_ubts->nbs, s);
				Y_Z[y_ics] = Y_Z[y_ics_0];
				y_ics->set_disambiguation_state(y_ics_0->get_disambiguation_state());
				terminal_state = y_ics;
			}
		}
	/* Add transitions from Z_UBTS states to Z_UBTS states*/
	for (Z_UBTS* z_ubts : ubts.ZSL) {
		push_internal_transitions(z_ubts->get_ZS(), z_ubts->CD,
								  z_ubts->num_predecessors);
		/* Add transitions from Z_UBTS states to Y_UBTS states*/
		for (auto& pair : z_ubts->children)
			if (z_ubts->CD[pair.first] || !fsm->controllable.at(pair.first))
				push_external_transitions(z_ubts->get_ZS(),
										  pair.second->get_YS(), pair.first,
										  z_ubts->num_predecessors,
										  pair.second->num_predecessors,
										  pair.second->disambiguation_state);
	}
	root_ics = get_ptr[ICS_STATE(ubts.nbaic->YSL[0], 0)];
}

/* Copy Constructor */
ICS::ICS(const ICS& other)
	: fsm(other.fsm), os(other.os), terminal_state(other.terminal_state) {
	copy_memory(other);
}

/* Assignment Operator */
ICS& ICS::operator=(const ICS& other) {
	if (this == &other) return *this;
	delete_memory();
	copy_memory(other);
}

ICS::~ICS() {
	for (auto& pair : get_ptr)
		delete pair.second;
}


///////////////////////////////////////////////////////////////////////////////


void ICS::push(YS* ys, ZS* zs, const CONTROL_DECISION& CD,
			   int p_index /*= 0*/, int c_index /*= 0*/,
			   UBTS_State* disambiguation_state /* = nullptr */) {
	/* Split YS into seperate states for each member of IS */
	for (STATE s = 0; s < ys->IS.size(); ++s)
		if (ys->IS[s]) push_helper(ys, s, zs, s, Y_Z, CD, p_index,
								   c_index, disambiguation_state);
}

void ICS::push(ZS* zs, NBAIC_State* nbs,
			   vector<tuple<STATE, EVENT, STATE>>& transitions,
			   UBTS_State* disambiguation_state /* = nullptr */) {
	/* Split ZS into seperate states for each member of IS */
	for (auto& transition : transitions)
		push_helper(zs, get<0>(transition),
					nbs, get<2>(transition),
					Z_YZ, get<1>(transition), 0, 0,
					disambiguation_state);
}

void ICS::pop(NBAIC_State* nbs, STATE s) {
	/* Find the pointer to the ICS_STATE matching input args */
	ICS_STATE ics_state(nbs, s);
	ICS_STATE* ics_ptr = get_ptr[ics_state];
	if (ics_ptr->is_Y_ICS()) {
		if (ics_ptr == root_ics) root_ics = nullptr;
		/* Delete any links to child states */
		for (auto& child : Y_Z[ics_ptr])
			delete_child_link(child.second, ics_ptr);
		/* Delete any links to parent states */
		for (ICS_STATE* parent : reverse[ics_ptr])
			delete_parent_link(find_link(Z_YZ[parent], ics_ptr),
							   Z_YZ, parent);
		/* Delete this state */
		Y_Z.erase(ics_ptr);
	}
	else {
		/* Delete any links to child states */
		for (auto& child : Z_YZ[ics_ptr])
			delete_child_link(child.second, ics_ptr);
		/* Delete any links to parent states */
		for (ICS_STATE* parent : reverse[ics_ptr]) {
			if (parent->is_Y_ICS())
				delete_parent_link(find_link(Y_Z[parent], ics_ptr),
								   Y_Z, parent);
			else
				delete_parent_link(find_link(Z_YZ[parent], ics_ptr),
								   Z_YZ, parent);
		}
		/* Delete this state */
		Z_YZ.erase(ics_ptr);
	}
	reverse.erase(ics_ptr);
	get_ptr.erase(ics_state);
	delete ics_ptr;
}

/* Returns true if there exists a Y-ICS State that is not coaccessible
(i.e., cannot reach a marked ICS state) */
bool ICS::exists_livelock(bool& root_is_coaccessible) {
	unordered_map<ICS_STATE*, int> ics_index = create_ics_index();

	if (ics_index.empty()) {
		root_is_coaccessible = false;
		return false;
	}
	vector<bool> coaccessible(ics_index.size(), false);

	for (auto iter = Y_Z.begin(); iter != Y_Z.end(); ) {
		ICS_STATE* ics_state = (*iter++).first;

		/* Previously found to be coaccessible in the path of another state */
		if (coaccessible[ics_index[ics_state]]) continue;

		vector<bool> visited(ics_index.size(), false);
		queue<Node<ICS_STATE*>*> BFS;
		Node<ICS_STATE*>* root = new Node<ICS_STATE*>(ics_state);
		BFS.push(root);
		livelock_BFS(BFS, root, coaccessible, visited, ics_index);

		/* BFS found no path to a marked state--must be livelocked */
		if (!coaccessible[ics_index[root->val]]) {
			root->val->get_nbs()->deleted = true;
			reset_tree(root);
			return true;
		}
		reset_tree(root);
	}
	return false;
}

ICS_STATE* ICS::get_entrance_state(UBTS& ubts) {
	unordered_map<ICS_STATE*, int> ics_index = create_ics_index();
	vector<bool> coaccessible(ics_index.size());

	for (auto iter = Y_Z.begin(); iter != Y_Z.end(); ) {
		ICS_STATE* ics_state = (*iter++).first;

		/* Previously found to be coaccessible in the path of another state */
		if (coaccessible[ics_index[ics_state]]) continue;

		vector<bool> visited(ics_index.size());
		queue<Node<ICS_STATE*>*> BFS;
		Node<ICS_STATE*>* root = new Node<ICS_STATE*>(ics_state);
		BFS.push(root);
		livelock_BFS(BFS, root, coaccessible, visited, ics_index);
		reset_tree(root);
	}
	/* All states coaccessible--no CELC exists */
	if (find(coaccessible.begin(), coaccessible.end(), false)
		  == coaccessible.end()) return nullptr;
	return find_CELC(ics_index, coaccessible, ubts);
}

/* Return event that links parent to child in ICS */
EVENT ICS::get_event(ICS_STATE* child, ICS_STATE* parent) {
	for (auto& pair : Z_YZ[parent])
		if (pair.second == child) return pair.first;
}

/* Return CD that links parent to child in ICS */
CONTROL_DECISION ICS::get_CD(ICS_STATE* child, ICS_STATE* parent) {
	for (auto& pair : Y_Z[parent])
		if (pair.second == child) return pair.first;
}

ICS_STATE* ICS::get_root() {
	if (root_ics) {return root_ics;}
	for (auto& pair : Y_Z) {
		ICS_STATE* ics_state = pair.first;
		if (ics_state->get_state() == 0 &&
			ics_state->get_transient_index() == 0 &&
			ics_state->get_nbs()->IS[0] &&
			find(ics_state->get_nbs()->IS.begin() + 1,
				 ics_state->get_nbs()->IS.end(), true) ==
				 ics_state->get_nbs()->IS.end()) {
			root_ics = ics_state;
			return ics_state;
		}
	}
	return nullptr;
}

void ICS::print() {
	os << "*******************************************************************\n"
	   << "*********************Inter-Connected System************************\n"
	   << "*******************************************************************\n";
	os << "Y-ICS States:\n";
	for (auto& transition : Y_Z) {
		os << "\tY-ICS State: {{"
		   << get_subset_string(transition.first->get_nbs()->IS, fsm->states)
		   << "}," << fsm->states.get_key(transition.first->get_state()) << ','
		   << transition.first->get_transient_index() << "}\n"
		   << (transition.second.empty() ? "" : "\tTransitions to:\n");
		for (auto& pair : transition.second)
			os << "\t\tZ-ICS State {{"
		   	   << get_subset_string(pair.second->get_nbs()->IS, fsm->states)
			   << "}," << fsm->states.get_key(pair.second->get_state()) << ','
			   << pair.second->get_transient_index()
			   << "} via Control Decision {"
			   << get_subset_string(pair.first, fsm->events) << "}\n";
		os << '\n';
	}
	os << "Z-States:\n";
	for (auto& transition : Z_YZ) {
		os << "\tZ-ICS State: {{"
		   << get_subset_string(transition.first->get_nbs()->IS, fsm->states)
		   << "}," << fsm->states.get_key(transition.first->get_state()) << ','
		   << transition.first->get_transient_index() << "}\n"
		   << (transition.second.empty() ? "" : "\tTransitions to:\n");
		for (auto& pair : transition.second)
			os << "\t\tICS State {{"
			   << get_subset_string(pair.second->get_nbs()->IS, fsm->states)
			   << "}," << fsm->states.get_key(pair.second->get_state()) << ','
			   << pair.second->get_transient_index() << "} via Event "
			   << fsm->events.get_key(pair.first) << "\n";
		os << '\n';
	}
	os << "*******************************************************************\n"
	   << "*********************End Inter-Connected System********************\n"
	   << "*******************************************************************\n";
}

void ICS::print_fsm(const char* const filename) {
	ofstream os(filename);
	os << get_ptr.size() << "\r\n\r\n";
	for (auto& pair : Y_Z) {
		print_state(os, pair.first, pair.second.size());
		for (auto& child_pair : pair.second) {
			/* Print the event leading to the child state and the child state and
			consider all CDs to be controllable and observable */
			os << '{' << get_subset_string(child_pair.first, fsm->events) << "}	";
			child_pair.second->print_fsm(os, fsm);
			os << "	c	o" << "\r\n";
		}
		os << "\r\n";
	}
	for (auto& pair : Z_YZ) {
		print_state(os, pair.first, pair.second.size());
		for (auto& child_pair : pair.second) {
			/* Print the event leading to the child state and the child state and
			Use the input controllable/observable values for all events */
			os << fsm->events.get_key(child_pair.first) << "	";
			child_pair.second->print_fsm(os, fsm);
			os << '	' << (fsm->controllable[child_pair.first] ? "c" : "uc")
			   << '	' << (fsm->observable[child_pair.first] ? "o" : "uo")
			   << "\r\n";
		}
		os << "\r\n";
	}
	os.close();
}

void ICS::print_state(ostream& os, ICS_STATE* ics_state, int num_transitions) {
	/* Print parent state, whether it's a marked Z-state, and # of children */
	ics_state->print_fsm(os, fsm);
	os << '	' << (ics_state->is_Y_ICS()
	   	   		  ? false
	   	   		  : fsm->marked[ics_state->get_state()])
	   << '	' << num_transitions << "\r\n";
}

void ICS::print_A_UxG(UBTS& ubts, ofstream& file_out,
					  bool write_to_file, bool write_to_screen) {
	if (!(write_to_screen || write_to_file)) return;
	if (write_to_screen)
		os << "*******************************************************************\n"
		   << "******************Controlled Language Automaton********************\n"
		   << "*******************************************************************\n";
	if (write_to_file) file_out << get_A_UxG_size() << "\r\n";
	unordered_map<ICS_STATE*, int> ics_index = create_ics_index();
	vector<bool> visited(ics_index.size());
	/* Start at root ICS_STATE {{0},0} */
	ICS_STATE* root_ics = get_root();
	Node<ICS_STATE*>* root = new Node<ICS_STATE*>(root_ics);
	stack<Node<ICS_STATE*>*> DFS;
	DFS.push(root);
	while (!DFS.empty()) {
		Node<ICS_STATE*>* current = DFS.top();
		DFS.pop();
		/* Prevent infinite loops */
		if (visited[ics_index[current->val]]) continue;
		visited[ics_index[current->val]] = true;
		/* Handle all Z-States up to next Y-States */
		print_A_UxG_helper(ubts, DFS, current, visited, ics_index,
						   file_out, write_to_file, write_to_screen);
	}
	reset_tree(root);
	if (write_to_screen)
		os << "*******************************************************************\n"
	   	   << "*****************End Controlled Language Automaton*****************\n"
	   	   << "*******************************************************************\n";
}

void ICS::reduce_A_UxG(const char* file_in, const char* file_out) {
	FSM A_UxG_FSM(file_in);
	ofstream os(file_out);
	A_UxG_FSM.reduce(os);
	os.close();
}


///////////////////////////////////////////////////////////////////////////////


void ICS::delete_memory() {
	/* Clean up dynamic memory */
	for (auto& pair : get_ptr)
		delete pair.second;
	/* Clean up unordered maps */
	Y_Z.clear();
	Z_YZ.clear();
	reverse.clear();
}

void ICS::copy_memory(const ICS& other) {
	/* Memory not handled dynamically by ICS */
	fsm = other.fsm;
	/* Allocate new states to avoid double pointers */
	for (auto& pair : other.get_ptr) {
		get_ptr[pair.first] = new ICS_STATE(pair.first);
		if (other.terminal_state == pair.second)
			terminal_state = get_ptr[pair.first];
		if (other.root_ics == pair.second)
			root_ics = get_ptr[pair.first];
		get_ptr[pair.first]->set_disambiguation_state(pair.second->get_disambiguation_state());
	}
	/* Use newly allocated states to fill transitions from Y states */
	for (auto& pair : other.Y_Z) {
		unordered_map<CONTROL_DECISION, ICS_STATE*> transition;
		for (auto& other_transition : pair.second)
			transition[other_transition.first] = get_ptr[*(other_transition.second)];
		Y_Z[get_ptr[*(pair.first)]] = transition;
	}
	/* Use newly allocated states to fill transitions from Z states */
	for (auto& pair : other.Z_YZ) {
		unordered_map<EVENT, ICS_STATE*> transition;
		for (auto& other_transition : pair.second)
			transition[other_transition.first] = get_ptr[*(other_transition.second)];
		Z_YZ[get_ptr[*(pair.first)]] = transition;
	}
	/* Use newly allocated states to create links to parents */
	for (auto& pair : other.reverse) {
		vector<ICS_STATE*> parents;
		for (ICS_STATE* ics_state : pair.second)
			parents.push_back(get_ptr[*ics_state]);
		reverse[get_ptr[*(pair.first)]] = parents;
	}
}

ICS_STATE* ICS::get_ICS_STATE(NBAIC_State* nbs, STATE s, int index /*= 0*/) {
	ICS_STATE* ics_state = new ICS_STATE(nbs, s, index);
	/* No matching ICS_State in ICS--create a new one */
	if (get_ptr.find(*ics_state) == get_ptr.end()) {
		get_ptr[*ics_state] = ics_state;
		return ics_state;
	}
	/* ICS_STATE match found--delete our
	temporary and return the existing state */
	else {
		ICS_STATE* result = get_ptr[*ics_state];
		delete ics_state;
		return result;
	}
}

void ICS::push_internal_transitions(ZS* zs, CONTROL_DECISION CD,
									int index /*= 0*/,
									UBTS_State* disambiguation_state /* = nullptr */) {
	/* For all pairs (i, j) s.t. i, j exist in the information state
	of the Z-State */
	for (STATE i = 0; i < zs->IS.size(); ++i) {
		if (!zs->IS[i]) continue;
		for (STATE j = 0; j < zs->IS.size(); ++j) {
			if (!zs->IS[j]) continue;
			/* For each FSM transition from state i */
			for (auto transition : fsm->transitions[i])
				/* i transitions to j in the FSM via an allowed event */
				if (j == transition.second
					&& (CD[transition.first]
						|| !fsm->controllable[transition.first]))
					push_helper(zs, i, zs, j, Z_YZ,
								transition.first, index, index);
		}
	}
}

void ICS::push_external_transitions(ZS* zs, YS* ys, EVENT e,
									int parent_index /*= 0*/,
									int child_index /*= 0*/,
									UBTS_State* disambiguation_state /* = nullptr */) {
	/* For all pairs (i, j) s.t. i exists in the information state
	of the Z-State and j exists in the information state of the Y-State */
	for (STATE i = 0; i < zs->IS.size(); ++i) {
		if (!zs->IS[i]) continue;
		for (STATE j = 0; j < ys->IS.size(); ++j) {
			if (!ys->IS[j]) continue;
			/* i transitions to j via event e */
			auto& i_map = fsm->transitions[i];
			if (i_map.find(e) != i_map.end() && i_map[e] == j)
				push_helper(zs, i, ys, j, Z_YZ, e, parent_index,
							child_index, disambiguation_state);
		}
	}
}

template <typename ICS_MAP, typename CONTROL>
void ICS::push_helper(NBAIC_State* nbs_key, STATE s_key,
					  NBAIC_State* nbs_val, STATE s_val,
					  ICS_MAP& ics_map, CONTROL& control,
					  int index_key /*= 0*/, int index_val /*= 0*/,
					  UBTS_State* disambiguation_state /* = nullptr */) {
	ICS_STATE* key = get_ICS_STATE(nbs_key, s_key, index_key);
	ICS_STATE* val = get_ICS_STATE(nbs_val, s_val, index_val);
	/* key and val are within a Z_UBTS state or its multiple Y_UBTS parents */
	if (disambiguation_state) {
		if (key->is_Y_ICS()) key->set_disambiguation_state(disambiguation_state);
		else if (val->is_Y_ICS()) val->set_disambiguation_state(disambiguation_state);
	}
	/* Create link to child */
	ics_map[key][control] = val;
	/* Create child's link to parent */
	vector<ICS_STATE*>& vec = reverse[val];
	if (find(vec.begin(), vec.end(), key) == vec.end()) vec.push_back(key);
}

void ICS::delete_child_link(ICS_STATE* child, ICS_STATE* parent) {
	/* Find vector iterator matching parent in the child's vector */
	auto link_to_parent = find(reverse[child].begin(),
							   reverse[child].end(), parent);
	if (link_to_parent != reverse[child].end())
		reverse[child].erase(link_to_parent);
}

template <typename LINK, typename ICS_MAP>
void ICS::delete_parent_link(LINK link_to_child, ICS_MAP& ics_map,
							 ICS_STATE* parent) {
	ics_map[parent].erase(link_to_child);
}

template <typename CONTROL>
CONTROL ICS::find_link(unordered_map<CONTROL, ICS_STATE*,
					   hash<CONTROL>>& ics_map, ICS_STATE* ics_state) const {
	/* Return the key used to access the child ptr from the parent's map */
	for (auto& pair : ics_map)
		if (pair.second == ics_state) return pair.first;
}

void ICS::livelock_BFS(queue<Node<ICS_STATE*>*>& BFS, Node<ICS_STATE*>* root,
					   vector<bool>& coaccessible, vector<bool>& visited,
					   unordered_map<ICS_STATE*, int>& ics_index) {
	while (!BFS.empty()) {
		Node<ICS_STATE*>* current = BFS.front();
		BFS.pop();
		/* Prevent infinite loops */
		if (visited[ics_index[current->val]]) continue;
		visited[ics_index[current->val]] = true;

		if (current->val->is_Y_ICS()) /* Y_ICS States */
			check_state_livelock(BFS, current, Y_Z,
								 coaccessible, visited, ics_index);
		else /* Z_ICS States */
			check_state_livelock(BFS, current, Z_YZ,
								 coaccessible, visited, ics_index);
		/* BFS found a path to a marked state--go to next state */
		if (coaccessible[ics_index[root->val]]) break;
	}
}

template <typename ICS_MAP>
void ICS::check_state_livelock(queue<Node<ICS_STATE*>*>& BFS,
							   Node<ICS_STATE*>* current,
							   ICS_MAP& ics_map, vector<bool>& coaccessible,
							   vector<bool>& visited,
							   unordered_map<ICS_STATE*, int>& ics_index) {
	for (auto& pair : ics_map[current->val]) {
		Node<ICS_STATE*>* next = new Node<ICS_STATE*>(pair.second);
		next->prev = current;
		current->next.push_back(next);
		/* We reached either a coaccessible Y-state or a marked Z-state,
		 so our state path must be coaccessible */
		if ((next->val->is_Y_ICS() && coaccessible[ics_index[next->val]])
			|| (!next->val->is_Y_ICS() && next->val->is_marked(fsm)))
			record_path(coaccessible, current, ics_index);
		/* Unmarked state not yet determined to be coaccessible--keep going */
		else BFS.push(next);
	}
}

/* Map each Y-ICS State to a unique int */
unordered_map<ICS_STATE*, int> ICS::create_ics_index() {
	unordered_map<ICS_STATE*, int> ics_index;
	int count = 0;
	if (!get_root()) return ics_index;
	Node<ICS_STATE*>* root = new Node<ICS_STATE*>(get_root());
	queue<Node<ICS_STATE*>*> BFS; /* Breadth first search */
	BFS.push(root);
	while (!BFS.empty()) {
		Node<ICS_STATE*>* current = BFS.front();
		BFS.pop();
		/* Prevent infinite loops & add to map */
		if (ics_index.find(current->val) != ics_index.end()) continue;
		ics_index[current->val] = count++;
		/* Add children to queue */
		if (current->val->is_Y_ICS()){
			for (auto& child_pair : Y_Z[current->val])
				BFS.push(make_child(current, child_pair.second));
		}
		else{
			for (auto& child_pair : Z_YZ[current->val])
				BFS.push(make_child(current, child_pair.second));
		}
	}
	return ics_index;
}

ICS_STATE* ICS::find_CELC(unordered_map<ICS_STATE*, int>& ics_index,
						  vector<bool>& coaccessible, UBTS& ubts) {
	/* For all non-coaccessible Y-States */
	for (auto iter = Y_Z.begin(); iter != Y_Z.end(); ) {
		ICS_STATE* ics_state = (*iter++).first;
		if (coaccessible[ics_index[ics_state]]) continue;

		vector<bool> visited(ics_index.size(), false);
		Node<ICS_STATE*>* root = new Node<ICS_STATE*>(ics_state);
		stack<Node<ICS_STATE*>*> DFS; /* Depth First Search */
		DFS.push(root);
		while(!DFS.empty()) {
			Node<ICS_STATE*>* current = DFS.top();
			DFS.pop();
			
			int index = ics_index[current->val];
			if (current->val->is_Y_ICS()) {
				/* Second time hitting this Y-State--we must have a cycle of 
				non-coaccessible states */
				if (visited[index]) {
					/* Cycle is a CELC--return entrance state */
					if (ICS_STATE* result = CELC_entrance_state(current, ubts)) {
						reset_tree(root);
						return result;
					}
					/* Cycle is not a CELC--continue from closest
					predecessor with > 1 child */
					else {
						backtrack(current);
						continue;
					}
				}
				/* First time hitting this Y-State */
				else {
					visited[index] = true;
					find_next_state(DFS, current, Y_Z[current->val]);
				}
			}
			else {
				if (visited[index]) continue;
				visited[index] = true;
				find_next_state(DFS, current, Z_YZ[current->val]);
			}
		}
		reset_tree(root);
	}
}

ICS_STATE* ICS::CELC_entrance_state(Node<ICS_STATE*>* end, UBTS& ubts) {
	Node<ICS_STATE*>* current = end;
	do {
		/* Check all Y-States in cycle. If one is transitioned to
		by an observable event then it is the entrance state */
		if (current->val->is_Y_ICS())
			for (auto& pair : Z_YZ[current->prev->val])
				if (pair.second == current->val
					&& fsm->observable[pair.first]
					&& ubts.is_terminal_Y_UBTS(current->val))
					return current->val;
		current = current->prev;
	} while (current->prev && current->val != end->val);
	/* No observable event transitioning to a Y-State--cycle is not a CELC */
	return nullptr;
}

int ICS::get_ICS_size(bool include_Y_ICS /* = true*/,
					  bool include_Z_ICS /* = true*/) {
	if (!include_Z_ICS && !include_Y_ICS) return 0;
	/* A_UxG size is equal to number of Z_ICS states */
	int Y_ICS_count = 0, Z_ICS_count = 0;
	unordered_map<ICS_STATE*, int> ics_index = create_ics_index();
	vector<bool> visited(ics_index.size());
	/* Start at root ICS_STATE {{0},0} */
	Node<ICS_STATE*>* root = new Node<ICS_STATE*>(get_root());
	queue<Node<ICS_STATE*>*> BFS; /* Breadth First Search */
	BFS.push(root);
	while (!BFS.empty()) {
		Node<ICS_STATE*>* current = BFS.front();
		BFS.pop();
		/* Prevent infinite loops */
   		if (visited[ics_index[current->val]]) continue;
   		visited[ics_index[current->val]] = true;
		/* Push children and increment if Z_ICS */
		if (current->val->is_Y_ICS()) {
			++Y_ICS_count;
			for (auto& child_pair : Y_Z[current->val])
				BFS.push(make_child(current, child_pair.second));
		}
		else {
			++Z_ICS_count;
			for (auto& child_pair : Z_YZ[current->val])
				BFS.push(make_child(current, child_pair.second));
		}
	}
	reset_tree(root);
	return (include_Y_ICS ? Y_ICS_count : 0) + (include_Z_ICS ? Z_ICS_count : 0);
}

int ICS::get_A_UxG_size() { return get_ICS_size(false); }

void ICS::print_A_UxG_helper(UBTS& ubts, stack<Node<ICS_STATE*>*>& DFS,
							 Node<ICS_STATE*>* current_ys, vector<bool>& visited,
							 unordered_map<ICS_STATE*, int>& ics_index,
							 ofstream& file_out, bool write_to_file,
							 bool write_to_screen) {
	vector<Node<ICS_STATE*>*> y_ics_states;
	/* Add first layer of Z-States */
	for (auto& pair : Y_Z[current_ys->val])
		DFS.push(make_child(current_ys, pair.second));
	/* For all Z-States between current Y-State and any subsequent Y-States */
	while (!DFS.empty() && !DFS.top()->val->is_Y_ICS()) {
		Node<ICS_STATE*>* current = DFS.top();
		DFS.pop();
		/* Prevent infinite loops */
   		if (visited[ics_index[current->val]]) continue;
   		visited[ics_index[current->val]] = true;

   		print_A_UxG_state(current_ys->val->get_nbs(),
   						  current->val->get_state(),
   						  Z_YZ[current->val].size(),
   						  current_ys->val->get_transient_index(),
   						  file_out, write_to_file, write_to_screen);
   		for (auto& pair : Z_YZ[current->val]) {
   			ICS_STATE* child = pair.second;
   			if (child->is_Y_ICS()) {
   				NBAIC_State* nbs = child->get_nbs();
   				int next_ics_index = ubts.is_terminal_Y_UBTS(child)
   									 ? 0
   									 : child->get_transient_index();
   				/* Grandchild is a Z-State with multiple Y-State
   				parents--current child Y-State will not be used */
   				UBTS_State* ds = child->get_disambiguation_state();
   				if (ds) {
   					nbs = ds->nbs;
   					next_ics_index = ds->num_predecessors;
   				}
	   			/* IS changes to the IS in the NBAIC_STATE of the new Y_ICS */
		   		print_A_UxG_transition(nbs, child->get_state(),
		   						 	   pair.first, ubts.is_terminal_Y_UBTS(child),
		   						 	   next_ics_index,
		   						 	   file_out, write_to_file,
		   						 	   write_to_screen);
		   		/* Save ICS_STATE for later */
   				if (!ubts.is_terminal_Y_UBTS(child) && !ds)
   					y_ics_states.push_back(make_child(current, child));
   			}
   			else {
   				/* IS used is the last seen Y_ICS NBAIC_STATE (i.e., current_ys) */
		   		print_A_UxG_transition(current_ys->val->get_nbs(),
		   							   child->get_state(), pair.first, false,
		   						 	   current_ys->val->get_transient_index(),
		   						 	   file_out, write_to_file,
		   						 	   write_to_screen);
		   		/* Add next layer of Z-states */
				DFS.push(make_child(current, child));
   			}
   		}
	}
	/* Append next layer of Y-States */
	for (auto& y_ics : y_ics_states) DFS.push(move(y_ics));
}

void ICS::print_A_UxG_state(NBAIC_State* nbs, STATE s, int transitions,
							int index, ofstream& file_out,
							bool write_to_file, bool write_to_screen) {
	if (write_to_screen)
		os << "A_UxG State {{" << get_subset_string(nbs->IS, fsm->states)
		   << "}," << fsm->states.get_key(s) << ',' << index << '}' << (fsm->marked[s] ? " m" : "")
		   << '\n' << (transitions ? "\tTransitions to:\n" : "");
	if (write_to_file)
		file_out << "\r\n{{" << get_subset_string(nbs->IS,fsm->states)
	   			<< "}," << fsm->states.get_key(s) << ',' << index << "}	"
	   			<< (fsm->marked[s] ? "1	" : "0	") << transitions << "\r\n";
}

void ICS::print_A_UxG_transition(NBAIC_State* nbs, STATE s,
								 EVENT e, bool is_terminal, int index,
								 ofstream& file_out, bool write_to_file,
								 bool write_to_screen) {
	if (write_to_screen)
		os << "\t\tA_UxG State {{" << get_subset_string(nbs->IS, fsm->states)
	       << "}," << fsm->states.get_key(s) << ','
	       << (is_terminal ? 0 : index) << "} via Event "
	       << fsm->events.get_key(e) << "\n";
    if (write_to_file)
    	file_out << fsm->events.get_key(e) << "	{{"
    			 << get_subset_string(nbs->IS, fsm->states)
		         << "}," << fsm->states.get_key(s) << ','
		         << (is_terminal ? 0 : index) << "}	"
		         << (fsm->controllable[e] ? "c	" : "uc	")
		         << (fsm->observable[e] ? "o" : "uo") << "\r\n";
}

bool ICS::is_terminal(ICS_STATE* s) {
	if (s->get_transient_index() == 0) return false;
	for (auto& pair : Y_Z[s])
		if (pair.second->get_transient_index() == 0) return true;
}

/* Z_ICS state has multiple Y_ICS parents, but we only choose one */
ICS_STATE* ICS::disambiguate(const ICS_STATE* const ics_state) {
	/* Find the Y_UBTS containing information for
	resolving the ambiguous state */
	UBTS_State* parent_Y_UBTS = ics_state->get_disambiguation_state();
	ICS_STATE disambiguated_state(parent_Y_UBTS->nbs, ics_state->get_state(),
								  parent_Y_UBTS->num_predecessors);
	/* Find current representation of the chosen parent already in the ICS */
	return get_ptr[disambiguated_state];
}

///////////////////////////////////////////////////////////////////////////////


void ICS_STATE::print(ostream& os) const {
	os << "ICS_State contents:\n"
	   << "\tNBAIC_State contents:\n"
	   << "\t\t deleted: " << nbs->deleted << '\n'
	   << "\t\t is_YS: " << is_Y_ICS() << '\n'
	   << "\t\t IS: ";
	for (bool b : nbs->IS) os << b << ' ';
	os << "\n\tState: " << s << '\n';
	os << "\tIndex: " << transient_index << '\n';
}

void ICS_STATE::print_fsm(ostream& os, FSM* fsm) {
	if (is_Y_ICS())
		os << "{{" << get_subset_string(get_nbs()->IS, fsm->states) << "},";
	else {
		ZS* zs = dynamic_cast<ZS*>(get_nbs());
		const CONTROL_DECISION* CD = nullptr;
		for (auto& pair : zs->reverse) {
			CD = &(pair.first);
			break;
		}
		os << "{{{" << get_subset_string(zs->IS, fsm->states)
		   << "},{" << get_subset_string(*CD, fsm->events) << "}},";
	}
	os << fsm->states.get_key(get_state()) << ',' << get_transient_index() << '}';
}

/* Two ICS_STATEs are equal if they point to the same
   NBAIC_STATE and store the same FSM state */
bool ICS_STATE::operator==(const ICS_STATE& other) const {
	return nbs == other.nbs && s == other.s;
}

size_t hash<ICS_STATE>::operator()(const ICS_STATE& ics_state) const {
    // Compute individual hash values for three data members
    // and combine them using XOR and bit shifting
    return ((hash<NBAIC_State*>()(ics_state.get_nbs())
    	  ^ (hash<STATE>()(ics_state.get_state()) << 1)
    	  ^ (hash<int>()(ics_state.get_transient_index()) << 2)) >> 1
    	  ^ (ics_state.is_Y_ICS()) << 3);
}


///////////////////////////////////////////////////////////////////////////////


template <typename BOOL_CONTAINER>
void record_path(BOOL_CONTAINER& bc, Node<ICS_STATE*>* current,
				 unordered_map<ICS_STATE*, int>& ics_index) {
	if (current->val->is_Y_ICS()) bc[ics_index[current->val]] = true;
	if (!current->prev) return;
	else record_path(bc, current->prev, ics_index);
}

template <typename ICS_MAP>
void find_next_state(stack<Node<ICS_STATE*>*>& DFS, Node<ICS_STATE*>* current,
					 ICS_MAP& ics_map) {
	/* No subsequent state exists */
	if (ics_map.empty()) {
		if (current->prev) backtrack(current);
		return;
	}
	/* At least one subsequent state exists--add them to the DFS */
	for (auto& pair : ics_map)
		push_child_to_stack(DFS, current, new Node<ICS_STATE*>(pair.second));
}

void push_child_to_stack(stack<Node<ICS_STATE*>*>& DFS,
						 Node<ICS_STATE*>* parent,
						 Node<ICS_STATE*>* child) {
	parent->next.push_back(child);
	child->prev = parent;
	DFS.push(child);
}

void backtrack(Node<ICS_STATE*>* current) {
	/* State does not transition--remove state and previous
	states up to last state with more than one child */
	do {
		Node<ICS_STATE*>* prev = current->prev;
		prev->next.erase(find(prev->next.begin(),
							  prev->next.end(),
							  current));
		delete current;
		current = prev;
	} while (current->next.size() == 0 &&
			 current->prev && current->prev->next.size() == 1);
}

Node<ICS_STATE*>* make_child(Node<ICS_STATE*>* current, ICS_STATE* child) {
	Node<ICS_STATE*>* next = new Node<ICS_STATE*>(child);
	next->prev = current;
	current->next.push_back(next);
	return next;
}


///////////////////////////////////////////////////////////////////////////////
