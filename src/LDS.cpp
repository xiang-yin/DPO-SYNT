#include <algorithm>
#include "../include/LDS.h"
#include "../include/Utilities.h"
using namespace std;


///////////////////////////////////////////////////////////////////////////////


/* Initial LDS is the shortest path from the entrance state
to the marked state--may not be maximal */
LDS::LDS(ostream& os_, NBAIC* nbaic_, ICS_STATE* entrance_state_)
	: os(os_), nbaic(nbaic_) {
	ICS& ics = nbaic->get_ics();
	ICS_STATE first_predecessor(entrance_state_->get_nbs(),
								entrance_state_->get_state());
	entrance_state = ics.get_ptr[first_predecessor];
	Node<ICS_STATE*>* root = new Node<ICS_STATE*>(entrance_state);
	queue<Node<ICS_STATE*>*> BFS; /* Breadth First Search */
	BFS.push(root);
	while(!BFS.empty()) {
		Node<ICS_STATE*>* current = BFS.front();
		BFS.pop();
		/* Found shortest path to marked state */
		if (current->val->is_marked(ics.fsm)) {
			record_path(ics, current);
			reverse(CDs.begin(), CDs.end());
			reverse(events.begin(), events.end());
			break;
		}
		if (current->val->is_Y_ICS()) push_children(BFS, ics.Y_Z, current);
		else push_children(BFS, ics.Z_YZ, current);
	}
	reset_tree(root);
}

/* Locally maximize all CDs s.t. no superset CD defined for a NBAIC_State
that maintains the LDS property for our path is not chosen */
void LDS::compute_maximal() {
	YS* current_YS = dynamic_cast<YS*>(entrance_state->get_nbs());
	for (int i = 0; i < CDs.size(); ++i) {
		/* Compare CD with all possible candidates defined at current state */
		for (auto& pair : current_YS->transition) {
			CONTROL_DECISION candidate = pair.first;
			if (is_subset(CDs[i], candidate) &&
				(events.empty() || is_LDS(current_YS, candidate, i)))
				CDs[i] = candidate;
		}
		/* Transition to next Y-State in path for iteration */
		ZS* current_ZS = current_YS->transition[CDs[i]];
		if (i < CDs.size() - 1) current_YS = current_ZS->transition[events[i]];
	}
}

void LDS::print() const {
	os << "*******************************************************************\n"
	   << "**********************Live Decision String*************************\n"
	   << "*******************************************************************\n";
	int i = 0;
	while (i < CDs.size() || i < events.size()) {
		if (i < CDs.size())
			os << "Control Decision " << i << ": {"
			   << get_subset_string(CDs[i], nbaic->get_fsm()->events) << "}\n";
		if (i < events.size())
			os << "Event " << i << ": " <<
			   nbaic->get_fsm()->events.get_key(events[i]) << '\n';
		++i;
	}
	os << "*******************************************************************\n"
	   << "**********************End Live Decision String*********************\n"
	   << "*******************************************************************\n";
}


///////////////////////////////////////////////////////////////////////////////


template <typename ICS_MAP>
void LDS::push_children(queue<Node<ICS_STATE*>*>& BFS, ICS_MAP& ics_map,
						Node<ICS_STATE*>* parent) {
	for (auto& pair : ics_map[parent->val]) {
		Node<ICS_STATE*>* child = new Node<ICS_STATE*>(pair.second);
		child->prev = parent;
		parent->next.push_back(child);
		BFS.push(child);
	}
}

void LDS::record_path(ICS& ics, Node<ICS_STATE*>* current) {
	/* We reached the root */
	if (!current->prev) return;
	/* Y_ICS is transitioned to by an observable event */
	else if (current->val->is_Y_ICS())
		events.push_back(ics.get_event(current->val, current->prev->val));
	/* Z_ICS is transitioned to by a Control Decision */
	else if (current->prev->val->is_Y_ICS())
		CDs.push_back(ics.get_CD(current->val, current->prev->val));
	record_path(ics, current->prev);
}

bool LDS::is_LDS(YS* current_YS, CONTROL_DECISION& CD, int index) {
	/* current_ZS is result of transitioning using our candidate CD */
	ZS* current_ZS = current_YS->transition[CD];
	/* Event events[index] in LDS no longer valid--
	LDS should not contain candidate CD */
	if (current_ZS->transition.find(events[index]) ==
		current_ZS->transition.end()) return false;
	/* events[index] still valid--keep going */
	else current_YS = current_ZS->transition[events[index++]];
	/* If the rest of the LDS is valid past the new CD, then adding the
	new CD will result in a more maximal LDS */
	for (; index < CDs.size(); ++index) {
		/* CD CDs[index] in LDS no longer valid--
		LDS should not contain candidate CD */
		if (current_YS->transition.find(CDs[index]) ==
			current_YS->transition.end()) return false;
		/* CDs[index] still valid--keep going */
		else current_ZS = current_YS->transition[CDs[index]];
		if (index < CDs.size() - 1) {
			/* Event events[index] in LDS no longer valid--
			LDS should not contain candidate CD */
			if (current_ZS->transition.find(events[index]) ==
				current_ZS->transition.end()) return false;
			/* events[index] still valid--keep going */
			else current_YS = current_ZS->transition[events[index]];
		}
	}
	/* All subsequent controls are valid past candidate CD--
	candidate should be used */
	return true;
}


///////////////////////////////////////////////////////////////////////////////
