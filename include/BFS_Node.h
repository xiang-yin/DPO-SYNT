#ifndef BFS_NODE_H
#define BFS_NODE_H

#include <vector>

/* Templated node class used in Breadth First Searches
   Contains pointer to a single parent and all children */
template <typename T>
struct Node {
	Node(const T& t_) : val(t_), prev(nullptr) {}
	const T val;
	Node<T>* prev;
	std::vector<Node<T>*> next;
};

#endif
