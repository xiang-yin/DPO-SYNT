#ifndef ICS_H
#define ICS_H

#include <tuple>
#include <queue>
#include <stack>
#include <iostream>
#include <fstream>
#include "Typedef.h"
#include "FSM.h"
#include "NBAIC_State.h"
#include "BFS_Node.h"
using namespace std;
/* Forward Declaration of Unfolded Bipartite Transition System */
class UBTS;
class UBTS_State;

/* Inter-Connected System State */
class ICS_STATE {
private:
	NBAIC_State* nbs;
	STATE s;
	int transient_index;
	UBTS_State* disambiguation_state;
public:
	ICS_STATE(NBAIC_State* nbs_, STATE s_)
		: nbs(nbs_), s(s_), transient_index(0), disambiguation_state(nullptr) {}
	ICS_STATE(NBAIC_State* nbs_, STATE s_, int transient_index_,
			  UBTS_State* disambiguation_state_ = nullptr)
		: nbs(nbs_), s(s_), transient_index(transient_index_),
		  disambiguation_state(disambiguation_state_) {}
	NBAIC_State* get_nbs() const { return nbs; }
	STATE get_state() const { return s; }
	int get_transient_index() const { return transient_index; }
	UBTS_State* get_disambiguation_state() const { return disambiguation_state; }
	void set_disambiguation_state(UBTS_State* ds_in) {disambiguation_state = ds_in; }
	bool is_Y_ICS() const {return nbs->is_YS; }
	bool is_marked(FSM* fsm) const { return fsm->marked[s] && !is_Y_ICS(); }
	void print(std::ostream& os) const;
	void print_fsm(std::ostream& os, FSM* fsm);
	bool operator==(const ICS_STATE& other) const;
};

/* ICS_STATE std::hash function */
namespace std {
    template <>
    struct hash<ICS_STATE> {
        size_t operator()(const ICS_STATE& ics_state) const;
    };
}

/* Inter-Connected System */
class ICS {
public:
	friend class NBAIC;
	friend class LDS;
	ICS(FSM* fsm_, std::ostream& os_)
		: fsm(fsm_), os(os_), terminal_state(nullptr) ,root_ics(nullptr){}
	ICS(UBTS& ubts, FSM* fsm_, std::ostream& os_);
	ICS(const ICS& other);
	ICS& operator=(const ICS& other);
	~ICS();
	void push(YS* ys, ZS* zs, const CONTROL_DECISION& CD,
			  int parent_index = 0, int child_index = 0,
			  UBTS_State* disambuguation_state = nullptr);
	void push(ZS* zs, NBAIC_State* nbs,
			  std::vector<std::tuple<STATE, EVENT, STATE>>& transitions,
			  UBTS_State* disambiguation_state  = nullptr );
	void pop(NBAIC_State* nbs, STATE s);
	bool exists_livelock(bool& root_is_coaccessible);
	ICS_STATE* get_entrance_state(UBTS& ubts);
	EVENT get_event(ICS_STATE* child, ICS_STATE* parent);
	CONTROL_DECISION get_CD(ICS_STATE* child, ICS_STATE* parent);
	ICS_STATE* get_root();
	int get_A_UxG_size();
	int get_ICS_size(bool include_Y_ICS = true,
					 bool include_Z_ICS = true);
	void print();
	void print_fsm(const char* const filename);
	void print_A_UxG(UBTS& ubts, std::ofstream& file_out,
					 bool write_to_file, bool write_to_screen);
	void reduce_A_UxG(const char* file_in, const char* file_out);
private:
	void delete_memory();
	void copy_memory(const ICS& other);
	ICS_STATE* get_ICS_STATE(NBAIC_State* nbs, STATE s, int index = 0);
	void push_internal_transitions(ZS* zs, CONTROL_DECISION CD, int index = 0,
								   UBTS_State* disambiguation_state = nullptr);
	void push_external_transitions(ZS* zs, YS* ys, EVENT e,
								   int parent_index = 0, int child_index = 0,
									UBTS_State* disambiguation_state = nullptr);
	template <typename ICS_MAP, typename CONTROL>
	void push_helper(NBAIC_State* nbs_key, STATE s_key,
					 NBAIC_State* nbs_val, STATE s_val,
					 ICS_MAP& ics_map, CONTROL& control,
					 int index_key = 0, int index_val = 0,
					 UBTS_State* disambiguation_state = nullptr);
	void delete_child_link(ICS_STATE* child, ICS_STATE* parent);
	template <typename LINK, typename ICS_MAP>
	void delete_parent_link(LINK link_to_child, ICS_MAP& ics_map,
							ICS_STATE* parent);
	template <typename CONTROL>
	CONTROL find_link(std::unordered_map<CONTROL, ICS_STATE*,
					  std::hash<CONTROL>>& ics_map, ICS_STATE* ics_state) const;
	void livelock_BFS(std::queue<Node<ICS_STATE*>*>& BFS, Node<ICS_STATE*>* root,
					  std::vector<bool>& coaccessible,
					  std::vector<bool>& visited,
					  std::unordered_map<ICS_STATE*, int>& ics_index);
	template <typename ICS_MAP>
	void check_state_livelock(std::queue<Node<ICS_STATE*>*>& BFS,
							  Node<ICS_STATE*>* current,
							  ICS_MAP& ics_map, std::vector<bool>& coaccessible,
							  std::vector<bool>& visited,
							  std::unordered_map<ICS_STATE*, int>& ics_index);
	std::unordered_map<ICS_STATE*, int> create_ics_index();
	ICS_STATE* find_CELC(std::unordered_map<ICS_STATE*, int>& ics_index,
						 std::vector<bool>& coaccessible, UBTS& ubts);
	ICS_STATE* CELC_entrance_state(Node<ICS_STATE*>* end, UBTS& ubts);
	void print_state(std::ostream& os, ICS_STATE* ics_state, int num_transitions);
	void print_A_UxG_helper(UBTS& ubts, std::stack<Node<ICS_STATE*>*>& BFS,
							Node<ICS_STATE*>* current_ys, std::vector<bool>& visited,
							std::unordered_map<ICS_STATE*, int>& ics_index,
							std::ofstream& file_out, bool write_to_file,
							bool write_to_screen);
	void print_A_UxG_state(NBAIC_State* nbs, STATE s,
						   int transitions, int index,
						   std::ofstream& file_out, bool write_to_file,
						   bool write_to_screen);
	void print_A_UxG_transition(NBAIC_State* nbs, STATE s, EVENT e,
								bool is_terminal, int index,
								std::ofstream& file_out, bool write_to_file,
								bool write_to_screen);
	bool is_terminal(ICS_STATE* s);
	ICS_STATE* disambiguate(const ICS_STATE* const ics_state);

	FSM* fsm; /* Finite State Machine */
	std::ostream& os;
	/* Find ICS_STATE ptr if one exists */
	std::unordered_map<ICS_STATE, ICS_STATE*> get_ptr;
	/* Y-State transition to Z-State */
	std::unordered_map<ICS_STATE*,
					   std::unordered_map<CONTROL_DECISION, ICS_STATE*,
										  std::hash<CONTROL_DECISION>>> Y_Z;
	/* Z-State transition to Y-State or Z-State */
	std::unordered_map<ICS_STATE*, std::unordered_map<EVENT, ICS_STATE*>> Z_YZ;
	/* Reverse lookup of which ICS-states
	transition to each other ICS_state */
	std::unordered_map<ICS_STATE*, std::vector<ICS_STATE*>> reverse;
	ICS_STATE* terminal_state;
	ICS_STATE* root_ics;
};

#endif
