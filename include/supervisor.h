#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include "CSR.h"
#include <unordered_map>
#include "Typedef.h"

/* Supervisor */
class SUPV : public DBTS{
public:
	SUPV(CSR* csr_);
	CSR* csr;
	std::unordered_map<YS*, Y_DBTS*> Y_req_map;
private:
	void DoDFS_sol(Y_DBTS* y, Y_DBTS* y_bar);
	bool if_contain(YS* y);
	CONTROL_DECISION get_max_R(YS* y,Y_DBTS* y_bar);
	CONTROL_DECISION get_max(YS* y);
	int get_numCD(ZS* z);
	int get_numIS(ZS* z);
	bool is_subset(const CONTROL_DECISION& CD1,const CONTROL_DECISION& CD2);
};

#endif
