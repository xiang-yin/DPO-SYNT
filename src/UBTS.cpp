#include <queue>
#include "../include/UBTS.h"
#include "../include/Utilities.h"
using namespace std;

static CONTROL_DECISION get_locally_maximal_CD(Y_UBTS* y_ubts, ZS*& zs);
template <typename State_Type, typename State_List>
bool is_duplicate(State_Type*& st, State_List& sl);


///////////////////////////////////////////////////////////////////////////////


UBTS_State::UBTS_State(NBAIC_State* nbs_, int num_predecessors_)
	: nbs(nbs_), num_predecessors(num_predecessors_),
	  disambiguation_state(nullptr) {}

bool UBTS_State::operator==(const UBTS_State& rhs) const {
	return nbs == rhs.nbs && num_predecessors == rhs.num_predecessors;
}


///////////////////////////////////////////////////////////////////////////////


UBTS::UBTS(NBAIC* nbaic_, ostream& os_) : nbaic(nbaic_), os(os_),
										  transient_state_index(-1),
										  terminal_Y_UBTS(nullptr) {
	YSL.push_back(new Y_UBTS(nbaic->YSL[0]));
}

UBTS::~UBTS() {
	for (Y_UBTS* y_ubts : YSL) delete y_ubts;
	for (Z_UBTS* z_ubts : ZSL) delete z_ubts; 
}


///////////////////////////////////////////////////////////////////////////////


void UBTS::expand() {
	while (exists_unvisited_Y_UBTS() || exists_unvisited_Z_UBTS()) {}
}

void UBTS::augment(LDS& lds) {
	/* Start adding states from the terminal state */
	Y_UBTS* next_Y = terminal_Y_UBTS;
	int i = 0;
	/* Alternate adding Z and Y states based on the controls in the LDS */
	while (i < lds.CDs.size()) {
		ZS* zs = next_Y->get_YS()->transition[lds.CDs[i]];
		Z_UBTS* next_Z = new Z_UBTS(zs, transient_state_index--, lds.CDs[i]);
		next_Y->child = next_Z;
		next_Z->parents.push_back(next_Y);
		ZSL.push_back(next_Z);

		if (i == lds.events.size()) break;

		YS* ys = next_Z->get_ZS()->transition[lds.events[i]];
		next_Y = new Y_UBTS(ys, transient_state_index--);
		next_Z->children[lds.events[i]] = next_Y;
		next_Y->parents.push_back(next_Z);
		YSL.push_back(next_Y);
		++i;
	}
	terminal_Y_UBTS = nullptr;
}

void UBTS::print() {
	os << "*******************************************************************\n"
	   << "**************Unfolded Bipartite Transition System*****************\n"
	   << "*******************************************************************\n";
	if (YSL.empty()) os << "No maximally permissive supervisor exists for this FSM\n";
	else {
		os << "Y-States:\n";
		for (Y_UBTS* y_ubts : YSL) {
			os << "\tY_UBTS State: {{"
			   << get_subset_string(y_ubts->get_YS()->IS, nbaic->fsm->states) << "},"
			   << y_ubts->num_predecessors << "}\n";
			if (y_ubts->child)
				os << "\tTransitions to:\n"
				   << "\t\tZ_UBTS State {{"
				   << get_subset_string(y_ubts->child->get_ZS()->IS, nbaic->fsm->states)
				   << "}," << y_ubts->child->num_predecessors
				   << "} via Control Decision {"
				   << get_subset_string(y_ubts->child->CD, nbaic->fsm->events)
				   << "}\n";
		}
		os << "Z-States:\n";
		for (Z_UBTS* z_ubts : ZSL) {
			os << "\tZ_UBTS State: {{"
			   << get_subset_string(z_ubts->get_ZS()->IS, nbaic->fsm->states) << "},"
			   << z_ubts->num_predecessors << "}\n"
			   << (z_ubts->children.empty() ? "" : "\tTransitions to:\n");
			for (auto& pair : z_ubts->children)
				os << "\t\tY_UBTS State {{"
				   << get_subset_string(pair.second->get_YS()->IS, nbaic->fsm->states)
				   << "}," << pair.second->num_predecessors
				   << "} via Event "
				   << nbaic->fsm->events.get_key(pair.first) << "\n";
		}
	}
	os << "*******************************************************************\n"
	   << "**************End Unfolded Bipartite Transition System*************\n"
	   << "*******************************************************************\n";
}

void UBTS::print(const char* const filename, bool print_ebts) {
	ofstream file_out(filename);
	file_out << YSL.size() + ZSL.size() - (print_ebts ? count_childless_Y_UBTS() : 0) 
			 << "\r\n\r\n";
	for (Y_UBTS* y_ubts : YSL) {
		if (print_ebts && !y_ubts->child) continue;
		file_out << "{{" << get_subset_string(y_ubts->get_YS()->IS,
											  nbaic->fsm->states) << "},"
				 << y_ubts->num_predecessors << "}	0	"
				 << (y_ubts->child ? "1" : "0") << "\r\n";
		if (y_ubts->child)
			file_out << '{' << get_subset_string(y_ubts->child->CD,
												 nbaic->fsm->events) << "}	{{"
					 << get_subset_string(y_ubts->child->get_ZS()->IS,
					 					  nbaic->fsm->states) << "},{"
					 << get_subset_string(y_ubts->child->CD,
					 					  nbaic->fsm->events) << "},"
					 << y_ubts->child->num_predecessors << "}	c	o\r\n";
		file_out << "\r\n";
	}
	for (Z_UBTS* z_ubts : ZSL) {
		file_out << "{{" << get_subset_string(z_ubts->get_ZS()->IS,
											 nbaic->fsm->states) << "},{"
				 << get_subset_string(z_ubts->CD, nbaic->fsm->events) << "},"
				 << z_ubts->num_predecessors << "}	1	"
				 << z_ubts->children.size() << "\r\n";
		for (auto& pair : z_ubts->children)
			file_out << nbaic->fsm->events.get_key(pair.first) << "	{{"
					 << get_subset_string(pair.second->get_YS()->IS,
					 					  nbaic->fsm->states) << "},"
					 << (print_ebts && !pair.second->child ?
					 	 0 : pair.second->num_predecessors) << "}	"
					 << (nbaic->fsm->controllable[pair.first] ? "c	" : "uc	")
					 << (nbaic->fsm->observable[pair.first] ? "o" : "uo")
					 << "\r\n";
		file_out << "\r\n";
	}
	file_out.close();
}


///////////////////////////////////////////////////////////////////////////////


bool UBTS::exists_unvisited_Y_UBTS() {
	for (Y_UBTS* y_ubts : YSL) {
		if (Y_UBTS_is_unvisited(y_ubts)) {
			visit_Y_UBTS(y_ubts);
			return true;
		}
	}
	return false;
}

bool UBTS::exists_unvisited_Z_UBTS() {
	for (Z_UBTS* z_ubts : ZSL) {
		if (Z_UBTS_is_unvisited(z_ubts)) {
			visit_Z_UBTS(z_ubts);
			return true;
		}
	}
	return false;
}

bool UBTS::Y_UBTS_is_unvisited(Y_UBTS* y_ubts) {
	/* Y_UBTS State is the first with its NBAIC_State and has
	no outgoing control decisions */
	return y_ubts->num_predecessors == 0 && !y_ubts->child;
}

bool UBTS::Z_UBTS_is_unvisited(Z_UBTS* z_ubts) {
	/* For all observable, controllable events in the control decision */
	for (EVENT e = 0; e < z_ubts->CD.size(); ++e)
		if (z_ubts->CD[e] && nbaic->fsm->observable[e]
			&& Z_UBTS_is_missing_transition(z_ubts, e)) return true;
	/* For all observable, uncontrollable events */
	for (EVENT e = 0; e < nbaic->fsm->uo.size(); ++e)
		if (Z_UBTS_is_missing_transition(z_ubts, nbaic->fsm->uo[e])) return true;
	/* No missing transitions */
	return false;
}

bool UBTS::Z_UBTS_is_missing_transition(Z_UBTS* z_ubts, EVENT e) {
	/* Event is defined for NBAIC_State and not for UBTS_State*/
	if (z_ubts->get_ZS()->transition.find(e)
		!= z_ubts->get_ZS()->transition.end()
		&& z_ubts->children.find(e)
		== z_ubts->children.end()) return true;
	/* Event is defined for both or neither */
	else return false;
}

void UBTS::visit_Y_UBTS(Y_UBTS* y_ubts) {
	ZS* zs = nullptr;
	CONTROL_DECISION CD = get_locally_maximal_CD(y_ubts, zs);
	/* Child Z-State of the passed-in y_ubts */
	Z_UBTS* child = new Z_UBTS(zs, CD);
	child->parents.push_back(y_ubts);
	child->num_predecessors = get_num_predecessors(child);
	if (!is_duplicate(child, ZSL)) ZSL.push_back(child);
	y_ubts->child = child;
}

void UBTS::visit_Z_UBTS(Z_UBTS* z_ubts) {
	/* For all observable, controllable events in the control decision */
	for (EVENT e = 0; e < z_ubts->CD.size(); ++e)
		if (z_ubts->CD[e] && nbaic->fsm->observable[e]
			&& Z_UBTS_is_missing_transition(z_ubts, e))
			add_child(z_ubts, e);
	/* For all observable, uncontrollable events */
	for (EVENT e = 0; e < nbaic->fsm->uo.size(); ++e)
		if (Z_UBTS_is_missing_transition(z_ubts, nbaic->fsm->uo[e]))
			add_child(z_ubts, nbaic->fsm->uo[e]);
}

void UBTS::add_child(Z_UBTS* z_ubts, EVENT e) {
	YS* ys = z_ubts->get_ZS()->transition[e];
	/* Child Y-State of the passed-in z_ubts */
	Y_UBTS* child = new Y_UBTS(ys);
	child->parents.push_back(z_ubts);
	child->num_predecessors = get_num_predecessors(child);
	if (!is_duplicate(child, YSL)) YSL.push_back(child);
	z_ubts->children[e] = child;
}

int UBTS::get_num_predecessors(UBTS_State* ubts_state) {
	unordered_map<UBTS_State*, int> UBTS_index = create_UBTS_index();
	vector<bool> visited(UBTS_index.size());
	queue<Node<UBTS_State*>*> BFS; /* Breadth First Search */
	Node<UBTS_State*>* root = new Node<UBTS_State*>(ubts_state);
	BFS.push(root);
	while (!BFS.empty()) {
		Node<UBTS_State*>* current = BFS.front();
		BFS.pop();
		/* Prevent infinite loops */
		if (visited[UBTS_index[current->val]]) continue;
		visited[UBTS_index[current->val]] = true;
		/* For each immediate parent state of the current node */
		for (UBTS_State* parent : current->val->parents) {
			/* NBAIC_States are the same and parent is not a transient state--
			root state's num_predecessors is one more than current's */
			if (ubts_state->nbs == parent->nbs
				&& parent->num_predecessors >= 0) {
				reset_tree(root);
				return parent->num_predecessors + 1;
			}
			/* NBAIC_States differ--keep searching */
			else {
				Node<UBTS_State*>* next = new Node<UBTS_State*>(parent);
				current->next.push_back(next);
				BFS.push(next);
			}
		}
	}
	reset_tree(root);
	/* BFS could not find matching, predecessor NBAIC_State--
	state is unique for this path */
	return 0;
}

/* Map each UBTS_State to a unique int */
unordered_map<UBTS_State*, int> UBTS::create_UBTS_index() {
	unordered_map<UBTS_State*, int> UBTS_index;
	UBTS_index.reserve(YSL.size() + ZSL.size());
	int count = 0;
	for (auto y_ubts : YSL) UBTS_index[y_ubts] = count++;
	for (auto z_ubts : ZSL) UBTS_index[z_ubts] = count++;
	return UBTS_index;
}

bool UBTS::is_terminal_Y_UBTS(ICS_STATE* ics_state) {
	NBAIC_State* nbs = ics_state->get_nbs();
	int index = ics_state->get_transient_index();
	for (Y_UBTS* y_ubts : YSL)
		/* Y_UBTS State is a terminal state with a matching NBAIC_State */
		if (y_ubts->nbs == nbs
			&& index == y_ubts->num_predecessors
			&& !y_ubts->child) {
			terminal_Y_UBTS = y_ubts;
			return true;
		}
	/* No matching terminal state exists */
	return false;
}

/* In the controlled language automaton, each z-state
must have a single y-state. If it does not, we redirect the ambiguous
y-states to a single parent */
void UBTS::mark_ambiguities() {
	for (Z_UBTS* z_ubts : ZSL) {
		if (z_ubts->parents.size() < 2) continue;
		for (int i = 1; i < z_ubts->parents.size(); ++i)
			z_ubts->parents[i]->disambiguation_state = z_ubts->parents.front();
	}
}

int UBTS::count_childless_Y_UBTS() {
	int result = 0;
	for (Y_UBTS* y_ubts : YSL)
		if (!y_ubts->child) ++result;
	return result;
}


///////////////////////////////////////////////////////////////////////////////


static CONTROL_DECISION get_locally_maximal_CD(Y_UBTS* y_ubts, ZS*& zs) {
	CONTROL_DECISION result(zs->nevents, false);
	for (auto& pair : y_ubts->get_YS()->transition)
		if (is_subset(result, pair.first)) {
			result = pair.first;
			zs = pair.second;
		}
	return result;
}

template <typename State_Type, typename State_List>
bool is_duplicate(State_Type*& st, State_List& sl) {
	for (auto state : sl) {
		/* State already in UBTS--delete new state and use existing */
		if (*st == *state) {
			state->parents.push_back(move(st->parents.front()));
			delete st;
			st = state;
			return true;
		}
	}
	/* No duplicate found in State List--use new state */
	return false;
}