#ifndef NBAIC_H
#define NBAIC_H

#include "ICS.h"
#include "IS_Property.h"
#include "BFS_Node.h"
#include "Typedef.h"

/* Non-Blocking All Inclusive Controller */
class NBAIC {
public:
	friend class UBTS;
	friend class ICS;
	friend class DBTS;
	friend class CSR;
	friend class SUPV;
	NBAIC(FSM* fsm_, IS_Property* isp_, std::ostream& os_,
		  Mode mode_ = BSCOPNBMAX);
	~NBAIC();
	void reduce_MPO(bool generate_maximal);
	ICS& get_ics() { return ics; }
	FSM* get_fsm() { return fsm; }
	bool is_empty() const { return ZSL.empty() || YSL.empty(); }
	void print(bool print_BDO = false);
	void print_fsm(const char* const filename);
private:
	FSM* fsm; /* Finite State Machine */
	IS_Property* ISP; /* Information-State Property */
	std::vector<YS*> YSL; /* Y-State List */
	std::vector<ZS*> ZSL; /* Z-State List */
	ICS ics; /* Inter-Connected System */
	std::ostream& os;
	Mode mode;

	void DoDFS_BSCOPNBMAX(YS* ys);
	void DoDFS_MPO(YS* ys);
	void DoDFS_MPRCP(YS* ys);
	SENSING_DECISION flag_observable(const SENSING_DECISION& SD);
	void prune();
	
	INFO_STATE unobservable_reach(const YS* ys, const CONTROL_DECISION& CD,
								  CONTROL_DECISION& used_events,
								  const std::vector<int>& max_CD,
								  std::vector<Transition>& UR_transitions);
	bool observable_reach(INFO_STATE& IS, const ZS* zs, const EVENT e,
						  std::vector<Transition>& OR_transitions);
	void find_next(std::queue<STATE>& BFS, const INFO_STATE& result,
				   const STATE current, const EVENT e,
				   std::vector<Transition>& UR_transitions);
	
	std::vector<int> get_max_CD(const YS* ys);
	ZS* get_ZS(const INFO_STATE& IS, const CONTROL_DECISION& CD, bool& zs_in_ZSL);
	YS* get_YS(const INFO_STATE& IS, bool& ys_in_YSL);
	bool ys_in_YSL(YS*& ys, ZS* zs, const INFO_STATE& IS, const EVENT e,
				   std::vector<Transition>& OR_transitions);
	CONTROL_DECISION get_CD(ZS* zs);
	
	template <typename State_List>
	void delete_NBAIC_States(State_List& sl);
	void delete_ICS_State(NBAIC_State* nbs);
	void delete_inaccessible();
	void mark_deleted(std::vector<bool>& accessible);
	void delete_states();

	bool redundant(const CONTROL_DECISION& CD,
				   const CONTROL_DECISION& used) const;
	bool is_deadlocked(const INFO_STATE& IS, const CONTROL_DECISION& CD,
					   const std::vector<int>& max_CD);
	void check_BSCOPNBMAX_deadlock(const std::vector<int>& max_CD,
								   std::queue<Node<STATE>*>& BFS,
								   Node<STATE>* current,
								   INFO_STATE& non_deadlocked,
								   const CONTROL_DECISION& CD);
	void check_MPO_deadlock(const std::vector<int>& max_SD,
							std::queue<Node<STATE>*>& BFS,
							Node<STATE>* current,
							INFO_STATE& non_deadlocked,
							const SENSING_DECISION& current_SD);
	bool exists_livelock();
	void check_state_deadlock(std::queue<Node<STATE>*>& BFS, Node<STATE>* current,
							  const EVENT e, INFO_STATE& non_deadlocked,
							  const SENSING_DECISION& current_SD);
	template <typename ICS_MAP>
	void check_state_livelock(std::queue<Node<ICS_STATE*>*>& BFS,
							  Node<ICS_STATE*>* current, ICS_MAP& ics_map,
							  std::vector<bool>& coaccessible,
							  std::vector<bool>& visited,
							  std::unordered_map<ICS_STATE*, int>& ics_index);
	
	std::unordered_map<NBAIC_State*, int> create_NBAIC_index(bool Y, bool Z) const;

	void print_sets(const INFO_STATE& IS, const CONTROL_DECISION& CD);
	std::string print_event_parameters(EVENT e);

	void run_tests();
	void deadlock_test(const INFO_STATE& IS);
	void opacity_test(const INFO_STATE& IS);
	void safety_test(const INFO_STATE& IS);
};

#endif
