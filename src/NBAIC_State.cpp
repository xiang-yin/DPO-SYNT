#include "../include/NBAIC_State.h"
using namespace std;

int NBAIC_State::nstates = 0;
int NBAIC_State::nevents = 0;

NBAIC_State::NBAIC_State(const bool deleted_, const bool is_YS_)
  : deleted(deleted_), is_YS(is_YS_) {}
NBAIC_State::NBAIC_State(const bool deleted_, const bool is_YS_,
						 const INFO_STATE& IS_)
  : deleted(deleted_), is_YS(is_YS_), IS(IS_) {}

string NBAIC_State::print() const {
	string result;
	result = (is_YS ? 'Y' : 'Z') + "-State with IS {";
	for (bool b : IS)
		result += b + ',';
	result += '}';
	return result;
}

YS::YS() : NBAIC_State(false, true) { IS.resize(nstates); }
YS::YS(const INFO_STATE& IS_) : NBAIC_State(false, true, IS_) {}

ZS::ZS() : NBAIC_State(false, false) { IS.resize(nstates); }
ZS::ZS(const INFO_STATE& IS_, CONTROL_DECISION CD_) : NBAIC_State(false, false, IS_), CD(CD_){}