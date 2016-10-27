#ifndef DES_UTILITIES_H
#define DES_UTILITIES_H

#include <string>
#include <vector>
#include "Bimap.h"
#include "BFS_Node.h"
#include "Typedef.h"

std::string get_subset_string(const std::vector<bool>& subset,
							  Bimap<std::string, int>& set);
bool is_subset(CONTROL_DECISION CD1, CONTROL_DECISION CD2);
void make_lower(char* str);

/* Recursively delete tree from the bottom up */
template <typename T> void reset_tree(Node<T>* current) {
	for (auto child : current->next) reset_tree(child);
	delete current;
}

#endif
