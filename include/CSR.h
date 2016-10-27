#ifndef CSR_H
#define CSR_H

#include <unordered_set>
#include <utility>
#include <unordered_map>
#include "DBTS.h"
#include "Typedef.h"

/* Control simulation relation */
class CSR_ypair{
public:
	YS* first;
	Y_DBTS* second;
	CSR_ypair(YS* y1, Y_DBTS* y2):first(y1),second(y2){}
};

class CSR_zpair{
public:
	ZS* first;
	Z_DBTS* second;
	CSR_zpair(ZS* z1, Z_DBTS* z2):first(z1),second(z2){}
};
class CSR{
public:
	std::unordered_set<CSR_ypair*> Y_map;
	std::unordered_set<CSR_zpair*> Z_map;
	std::unordered_map<YS*, std::unordered_map<Y_DBTS*, CSR_ypair*> > Y_pair_pointers;
	std::unordered_map<ZS*, std::unordered_map<Z_DBTS*, CSR_zpair*> > Z_pair_pointers;
	NBAIC* aic;
	DBTS* dbts;
	std::ostream& os;
	CSR(DBTS* bts_, ostream& os_);
	bool is_subset(const CONTROL_DECISION& CD1,const CONTROL_DECISION& CD2);
	bool Dosearch();
	void print();
};
#endif
