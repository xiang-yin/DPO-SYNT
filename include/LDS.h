#ifndef LDS_H
#define LDS_H

#include "NBAIC.h"
#include "Typedef.h"

class LDS { /* Live Decision String */
public:
	LDS(std::ostream& os_, NBAIC* nbaic_, ICS_STATE* entrance_state_);
	void compute_maximal();
	void print() const;
	std::vector<CONTROL_DECISION> CDs;
	std::vector<EVENT> events;
private:
	template <typename ICS_MAP>
	void push_children(std::queue<Node<ICS_STATE*>*>& BFS,
					   ICS_MAP& ics_map, Node<ICS_STATE*>* parent);
	void record_path(ICS& ics, Node<ICS_STATE*>* current);
	bool is_LDS(YS* current_YS, CONTROL_DECISION& CD, int index);
	std::ostream& os;
	NBAIC* nbaic;
	ICS_STATE* entrance_state;
};

#endif
