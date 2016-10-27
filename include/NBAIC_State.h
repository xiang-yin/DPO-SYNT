#ifndef NBAIC_STATE_H
#define NBAIC_STATE_H

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include "Typedef.h"

/* Non-Blocking All Inclusive Controller State */
class NBAIC_State {
public:
	NBAIC_State(const bool deleted_, const bool is_YS_);
	NBAIC_State(const bool deleted_, const bool is_YS_, const INFO_STATE& IS_);
	virtual ~NBAIC_State() {}
	std::string print() const;
	static int nstates;
	static int nevents;
	bool deleted;
	bool is_YS;
	INFO_STATE IS;
};

/* Forward Declarations */
class YS;
class ZS;

/* Y-state */
class YS : public NBAIC_State {
public:
	YS();
	YS(const INFO_STATE& IS_);
	virtual ~YS() {}
	/* Link to child states */
	std::unordered_map<CONTROL_DECISION, ZS*,
					   std::hash<CONTROL_DECISION>> transition;
	/* Link to parent states */
	std::unordered_map<EVENT, std::vector<ZS*>> reverse;
};

/* Z-state */
class ZS : public NBAIC_State {
public:
	ZS();
	ZS(const INFO_STATE& IS_, CONTROL_DECISION CD_);
	virtual ~ZS() {}
	/* Link to child states */
	std::unordered_map<EVENT, YS*> transition;
	/* Link to parent states */
	std::unordered_map<CONTROL_DECISION, std::vector<YS*>,
					   std::hash<CONTROL_DECISION>> reverse;
	CONTROL_DECISION CD;
};

#endif
