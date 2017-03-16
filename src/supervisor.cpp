#include <queue>
#include "../include/supervisor.h"
#include "../include/Utilities.h"
using namespace std;

SUPV::SUPV(CSR* csr_): DBTS(csr_->os){
	csr = csr_;
	aic = csr->aic;
	req_fsm = csr->dbts->req_fsm;
	req_states = csr->dbts->req_states;
	/* Create initial Y-State */
	Y_DBTS* y0 = new Y_DBTS(aic->YSL[0]);
	YSL.push_back(y0);
	for (auto i:csr->dbts->YSL){
		Y_req_map[i->get_YS()] = i;
	}
	DoDFS_sol(y0,csr->dbts->YSL[0]);
}


void SUPV::DoDFS_sol(Y_DBTS* y, Y_DBTS* y_bar){
	CONTROL_DECISION Act;
	YS* y_state = y->get_YS();

	if (if_contain(y_state) && y_bar){
		Act = get_max_R(y_state, y_bar);
	}
	else{
		Act = get_max(y_state);
	}
	
	bool judge;
	ZS* z_state = y_state -> transition[Act];
	/* get the real z and judge whether the z state is already in the DBTS*/

	Z_DBTS* z = get_Z_DBTS(z_state, Act, judge);
	y->child = z;
	z->parents.push_back(y);

	Z_DBTS* z_bar;
	if (y_bar){
		z_bar = y_bar -> child;
	}
	else{
		z_bar = nullptr;
	}
	if (!judge){
		ZSL.push_back(z);
		for (auto i:z_state->transition){
			bool judge_y;
			Y_DBTS* new_y = get_Y_DBTS(i.second, judge_y);
			z->children[i.first] = new_y;
			new_y->parents.push_back(z);
			Y_DBTS* new_y_bar;
			if ((z_bar) && (z_bar->children.find(i.first)!=z_bar->children.end())){
				new_y_bar = z_bar->children[i.first];
			} 
			else
				new_y_bar = nullptr;
			if (!judge_y){
				YSL.push_back(new_y);
				DoDFS_sol(new_y, new_y_bar);
			}
		}
	}

}

bool SUPV::if_contain(YS* y){
	for (STATE i = 0; i < aic->fsm->nstates; i++){
		if (y->IS[i] && req_states[i]){
			return true;
		}
	}
	return false;
}

CONTROL_DECISION SUPV::get_max_R(YS* y,Y_DBTS* y_bar){
	//get the CD with the biggest possiblities.
	int maxCD = -1;
	int maxIS = -1;
	CONTROL_DECISION result(aic->fsm->nevents, false);
	for (auto i:y->transition){
		if (y_bar->child){
			if (csr->Z_map.find(csr->Z_pair_pointers[i.second][y_bar->child])!=csr->Z_map.end()
				&& is_subset(i.second->CD, y_bar->child->CD)){
				int num_CD = get_numCD(i.second);
				int num_IS = get_numIS(i.second);
				if (maxCD < num_CD || maxIS < num_IS){
					maxCD = num_CD;
					maxIS = num_IS;
					result = i.first;
				}
			}
		}
	}
	return result;
}

CONTROL_DECISION SUPV::get_max(YS* y){
	//get the CD with the biggest possiblities.
	int maxCD = -1;
	int maxIS = -1;
	CONTROL_DECISION result;
	for (auto i:y->transition){
		int num_CD = get_numCD(i.second);
		int num_IS = get_numIS(i.second);
		if (maxCD < num_CD || maxIS < num_IS){
			maxCD = num_CD;
			maxIS = num_IS;
			result = i.first;
		}
	}
	return result;
}



int SUPV::get_numCD(ZS* z){
	//get the number of events in the CD
	int result = 0;
	for (auto i:z->CD)
		if (i)
			result ++;
    return result;
}

int SUPV::get_numIS(ZS* z){
	//get the number of states of the IS
	int result = 0;
	for (auto i:z->IS)
		if (i)
			result ++;
    return result;
}

bool SUPV::is_subset(const CONTROL_DECISION& CD1,const CONTROL_DECISION& CD2){
	//determine whether CD2 is a subset of CD1
	for (EVENT i = 0; i < aic->fsm->nevents; i++){
		if ((CD2[i])&&(!CD1[i]))
			return false;
	}
	return true;
}