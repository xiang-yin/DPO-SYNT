#ifndef IS_PROPERTY_H
#define IS_PROPERTY_H

#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include "Typedef.h"

/* Information-State Property */
class IS_Property {
public:
	virtual bool operator() (const INFO_STATE& IS) const = 0;
	virtual ~IS_Property() {}
};

/* Opacity--true iff IS is not a subset of secret_states */
class Opacity : public IS_Property {
public:
	Opacity(const std::vector<bool>& secret_states_);
	Opacity(const std::string& filename,
			std::unordered_map<std::string, STATE>& all_states);
	bool operator() (const INFO_STATE& IS) const;
private:
	std::vector<bool> secret_states;
};

/* Safety--true if no state in IS is also in unsafe_states */
class Safety : public IS_Property {
public:
	Safety();
	Safety(const std::vector<bool>& unsafe_states_);
	Safety(const std::string& filename,
		   std::unordered_map<std::string, STATE>& all_states);
	bool operator() (const INFO_STATE& IS) const;
	std::vector<bool> unsafe_states;
};

/* Disambiguation--true if IS does not contain
states in both A_states & B_states aa*/
class Disambiguation : public IS_Property {
public:
	Disambiguation(const std::vector<bool>& A_states_,
			  	   const std::vector<bool>& B_states_);
	Disambiguation(const std::string& filename,
		   	  	   std::unordered_map<std::string, STATE>& all_states);
	bool operator() (const INFO_STATE& IS) const;
private:
	std::vector<bool> A_states, B_states;
};

IS_Property* get_ISP(const std::string& property, const std::string& ISP_file,
					 std::unordered_map<std::string, STATE>& states, bool verbose);
IS_Property* get_ISP();

#endif
