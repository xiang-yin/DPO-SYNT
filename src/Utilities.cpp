#include "../include/Utilities.h"
using namespace std;

/* Returns a text representation of an information state or control decision */
string get_subset_string(const vector<bool>& subset, Bimap<string, int>& set) {
	string result;
	for (int i = 0; i < subset.size(); ++i)
		if (subset[i]) result.append(set.get_key(i) + ',');
	/* Remove last comma */
	if (!result.empty()) result.pop_back();
	return result;
}

/* Returns true if CD2 has at least all of the same active events as CD1 */
bool is_subset(CONTROL_DECISION CD1, CONTROL_DECISION CD2) {
	for (int i = 0; i < CD1.size(); ++i)
		if (CD1[i] && !CD2[i]) return false;
	return true;
}

/* Turns string str into a lowercase string using O(1) extra memory */
void make_lower(char* str) {
	int i = 0;
	while (str[i]) str[i] = tolower(str[i++]);
}