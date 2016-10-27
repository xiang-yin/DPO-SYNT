#ifndef UBTS_H
#define UBTS_H

#include "LDS.h"
#include "Typedef.h"

/* Unfolded Bipartite Transition System State */
class UBTS_State {
public:
	UBTS_State(NBAIC_State* nbs_, int num_predecessors_);
	virtual ~UBTS_State() {}
	bool operator==(const UBTS_State& rhs) const;
	bool is_Y_UBTS() const { return nbs->is_YS; }
	NBAIC_State* nbs;
	int num_predecessors;
	std::vector<UBTS_State*> parents;
	UBTS_State* disambiguation_state;
};

/* Forward Declarations */
class Y_UBTS;
class Z_UBTS;

/* Y-State */
class Y_UBTS : public UBTS_State {
public:
	Y_UBTS(YS* ys_) : UBTS_State(ys_, 0), child(nullptr) {}
	Y_UBTS(YS* ys_, int num_predecessors_)
		: UBTS_State(ys_, num_predecessors_), child(nullptr) {}
	virtual ~Y_UBTS() {}
	YS* get_YS() { return dynamic_cast<YS*>(nbs); }
	Z_UBTS* child;
};

/* Z-State */
class Z_UBTS : public UBTS_State {
public:
	Z_UBTS(ZS* zs_, CONTROL_DECISION& CD_) : UBTS_State(zs_, 0), CD(CD_) {}
	Z_UBTS(ZS* zs_, int num_predecessors_, CONTROL_DECISION& CD_)
		: UBTS_State(zs_, num_predecessors_), CD(CD_) {}
	virtual ~Z_UBTS() {}
	ZS* get_ZS() { return dynamic_cast<ZS*>(nbs); }
	std::unordered_map<EVENT, Y_UBTS*> children;
	CONTROL_DECISION CD;
};

/* Unfolded Bipartite Transition System */
class UBTS {
public:
	friend class ICS;
	UBTS(NBAIC* nbaic_, std::ostream& os_);
	~UBTS();
	void expand();
	void augment(LDS& lds);
	void print();
	void print(const char* const filename, bool print_ebts);
private:
	NBAIC* nbaic; /* Non-Blocking All Inclusive Controller */
	std::vector<Y_UBTS*> YSL; /* Y_UBTS State List */
	std::vector<Z_UBTS*> ZSL; /* Z_UBTS State List */
	std::ostream& os;
	int transient_state_index;
	Y_UBTS* terminal_Y_UBTS;

	bool exists_unvisited_Y_UBTS();
	bool exists_unvisited_Z_UBTS();
	bool Y_UBTS_is_unvisited(Y_UBTS* y_ubts);
	bool Z_UBTS_is_unvisited(Z_UBTS* z_ubts);
	bool Z_UBTS_is_missing_transition(Z_UBTS* z_ubts, EVENT e);
	void visit_Y_UBTS(Y_UBTS* y_ubts);
	void visit_Z_UBTS(Z_UBTS* z_ubts);
	void add_child(Z_UBTS* z_ubts, EVENT e);
	int get_num_predecessors(UBTS_State* root);
	std::unordered_map<UBTS_State*, int> create_UBTS_index();
	bool is_terminal_Y_UBTS(ICS_STATE* ics_state);
	void mark_ambiguities();
	int count_childless_Y_UBTS();
};

#endif
