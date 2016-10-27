#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <vector>
#include <tuple>

typedef int STATE;
typedef int EVENT;
typedef std::vector<bool> INFO_STATE;
typedef std::vector<bool> CONTROL_DECISION;
typedef std::vector<bool> SENSING_DECISION;
typedef std::vector<bool> REQUIRED_STATE;
typedef std::tuple<STATE, EVENT, STATE> Transition;

enum Mode {INTERACTIVE, BSCOPNBMAX, MPO, CONVERT, MPRCP};

#endif