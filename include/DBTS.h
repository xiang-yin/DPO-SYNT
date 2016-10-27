#ifndef DBTS_H
#define DBTS_H

#include "LDS.h"
#include "Typedef.h"

/* Determined Bipartite Transition System State */
class DBTS_State {
public:
	DBTS_State(NBAIC_State* nbs_): nbs(nbs_){}
	virtual ~DBTS_State() {}
	bool operator==(const DBTS_State& rhs) const;
	bool is_Y_DBTS() const { return nbs->is_YS; }
	NBAIC_State* nbs;
	std::vector<DBTS_State*> parents;
};

/* Forward Declarations */
class Y_DBTS;
class Z_DBTS;

/* Y-State */
class Y_DBTS : public DBTS_State {
public:
	Y_DBTS(YS* ys_) : DBTS_State(ys_), child(nullptr) {}
	virtual ~Y_DBTS() {}
	YS* get_YS() {return dynamic_cast<YS*>(nbs); }
	Z_DBTS* child;
};

/* Z-State */
class Z_DBTS : public DBTS_State {
public:
	Z_DBTS(ZS* zs_, CONTROL_DECISION CD_) : DBTS_State(zs_), CD(CD_) {}
	virtual ~Z_DBTS() {}
	ZS* get_ZS() { return dynamic_cast<ZS*>(nbs); }
	std::unordered_map<EVENT, Y_DBTS*> children;
	CONTROL_DECISION CD;
};

/* Determined Bipartite Transition System */
class DBTS {
public:
	friend class CSR;
	DBTS(ostream& os_): os(os_){}
	DBTS(NBAIC* aic_, FSM* req_fsm_, ostream& os_, bool& sol);
	~DBTS();
	void print();
	void print(const char* const filename);
	NBAIC* aic; /* All Inclusive Controller */
	std::vector<Y_DBTS*> YSL; /* Y_DBTS State List */
	std::vector<Z_DBTS*> ZSL; /* Z_DBTS State List */
	std::ostream& os;
	REQUIRED_STATE req_states;
	FSM* req_fsm;
	bool DoDFS(Y_DBTS* y);
	std::vector<STATE> get_states(YS* y);
	CONTROL_DECISION get_cd(const std::vector<STATE> y_req_states);
	Z_DBTS* get_Z_DBTS(ZS* z, CONTROL_DECISION Act, bool& judge);
	Y_DBTS* get_Y_DBTS(YS* y, bool& judge);

};

#endif
