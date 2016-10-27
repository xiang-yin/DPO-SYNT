#include "../include/CSR.h"
#include "../include/Utilities.h"
using namespace std;

CSR::CSR(DBTS* dbts_, ostream& os_)
	:dbts(dbts_), os(os_), aic(dbts_->aic){
	//Initializing Y_map;
	for (YS* i:aic->YSL)
		for (Y_DBTS* j:dbts->YSL){
			CSR_ypair* temp = new CSR_ypair(i,j);
			Y_map.insert(temp);
			Y_pair_pointers[i][j] = temp;
		}

	for (ZS* i:aic->ZSL)
		for (Z_DBTS* j:dbts->ZSL){
			CSR_zpair* temp = new CSR_zpair(i,j);
			Z_map.insert(temp);
			Z_pair_pointers[i][j] = temp;
		}

	while (Dosearch()){}//search for the total CSRs.
}

bool CSR::Dosearch(){
	bool need_to_delete = false;
	auto new_Ymap = Y_map;
	//delete pairs from Y_map to get new_Ymap.
	for (auto i:Y_map){

		YS* y1 = i->first;
		Y_DBTS* y2 = i->second;
		CONTROL_DECISION CD_y2 = y2->child->CD;
		Z_DBTS* z2 = y2->child;
		bool to_delete = true;

		for (auto j:y1->transition){
			if (is_subset(j.first,CD_y2) && (Z_map.find(Z_pair_pointers[j.second][z2]) != Z_map.end())){
				to_delete = false;
				break;
			}
		}
		if (to_delete){
			need_to_delete = true;
			new_Ymap.erase(i);

		}
	}
	//delete pairs from Z_map to get new_Zmap.
	auto new_Zmap = Z_map;
	for (auto i:Z_map){
		ZS* z1 = i->first;
		Z_DBTS* z2 = i->second;
		for (auto j:z2->children){
			if ((z1->transition.find(j.first) == z1->transition.end()) 
				|| (Y_map.find(Y_pair_pointers[z1->transition[j.first]][j.second]) == Y_map.end())){
				need_to_delete = true;
				new_Zmap.erase(i);
				break;
			}
		}
	}
    Z_map = new_Zmap;
    Y_map = new_Ymap;
    return need_to_delete;
}


bool CSR::is_subset(const CONTROL_DECISION& CD1,const CONTROL_DECISION& CD2){
	//determine whether CD2 is a subset of CD1
	for (EVENT i = 0; i < aic->fsm->nevents; i++){
		if ((CD2[i])&&(!CD1[i]))
			return false;
	}
	return true;
}

void CSR::print(){
	os << "*******************************************************************\n"
	       << "********************Control Simulation Relation********************\n"
	       << "*******************************************************************\n";
	os<<"Y state pairs:\n";
	for (auto i:Y_map){
		os<<"({"
		  <<get_subset_string(i->second->get_YS()->IS, aic->fsm->states)<<"},{"
		  <<get_subset_string(i->first->IS, aic->fsm->states)
		  <<"})\n";
	}
	os<<"Z state pairs:\n";
	for (auto i:Z_map){
		os<<"({{"
		  <<get_subset_string(i->second->get_ZS()->IS, aic->fsm->states)<<"},{"
		  <<get_subset_string(i->second->CD, aic->fsm->events)<<"}},{{"
		  <<get_subset_string(i->first->IS, aic->fsm->states)<<"},{"
		  <<get_subset_string(i->first->CD, aic->fsm->events)<<"}})\n";
		
	}
	os << "*******************************************************************\n"
	       << "********************Control Simulation Relation********************\n"
	       << "*******************************************************************\n";
}
