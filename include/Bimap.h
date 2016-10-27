#ifndef BIMAP_H
#define BIMAP_H

#include <unordered_map>

/* Templated Bi-directional Unordered Map class*/
template <typename T1, typename T2>
class Bimap {
public:
	std::unordered_map<T1, T2> regular;
	std::unordered_map<T2, T1> inverse;

	Bimap(){}
	Bimap(const int n) { reserve(n); }

	void insert(const T1 t1, const T2 t2) {
		regular[t1] = t2;
		inverse[t2] = t1;
	}

	int size() const { return regular.size(); }
	void reserve(const int n) {
		regular.reserve(n);
		inverse.reserve(n);
	}
	bool empty() const { return regular.empty(); }

	bool find_key(const T2& t2) const { return inverse.find(t2) != inverse.end(); }
	bool find_value(const T1& t1) const { return regular.find(t1) != regular.end(); }

	T1& get_key(const T2& t2) { return inverse[t2]; }
	T2& get_value(const T1& t1) { return regular[t1]; }

	T2& operator[] (const T1& t1) { return regular[t1]; }
};

#endif
