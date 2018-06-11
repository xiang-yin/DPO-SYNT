# DPO-SYNT: Discrete Control Synthesis for Partially-Observed Systems
## News
New model MPO is available to support the synthesis of centralized sensor activatio policy.

## GENERAL USAGE NOTES

DPO-SYNT is a C++ based software toolbox for property enforcement for partially-observed Discrete Event Systems. 
It implements a recently developed uniform framework for synthesizing maximally-permissive supervisors and optimal
sensor activation policies decribed in [1,2,3,4]. 
It can handle a large variety of properties, including safety, opacity, diagnosability, detectability, in a uniform manner.

This program contains implementations of BSCOPNBMAX, MPRCP and MPO, as well as a converter utility, defined as follows:

* BSCOPNBMAX: 
Basic Supervisory Control and Observation Problem: Non-blocking and Maximally Permissive Case 
is the problem of synthesizing non-blocking, deterministic, information-state property compliant,
maximally-permissive supervisors for a partially observed Discrete Event
System (DES). The problem as well as an algorithmic solution for BSCOPNBMAX are described in [1] and [2].

* MPO: 
Most Permissive Observer is a Bipartite Dynamic Observer used to solve
Dynamic Sensor Activation Problems for partially observed DES. The structure
is utilized in [3] to synthesize either a maximal
or minimal sensor activation policy for an arbitrary information state
property.

* MPRCP: 
Maximally-Permissive Supervisors for the Range Control Problem is a problem 
of synthesizing maximaaly-permissive superviors for a partially-observed Discrete
Event System (DES). For now, we consider only safety property. In this mode, we need three 
FSM inputs, which are G, the original FSM, K, the FSM contains only the safety states and R
, the FSM describes the required behavior. The language of R has to be the subset of the 
language of K. Also, the language of K has to be the subset of the language of G. The three 
FSMs must satisfy the requirement. This problem is described in [4].

* CONVERT will convert FSM files between the format listed below and the
UMDES .fsm file format, which may be useful for viewing structures in DESUMA


## Compiling DPO-SYNT:
 * Within this directory in the command terminal, run the command `make`.
   NOTE: this program uses features of C++ unique to C++-11 and later.
 * For Windows: you can run DPO-SYNT directly by double clicking `DES_Supervisor_win.exe`
 
## Running DPO-SYNT:

### Command line execution of DPO-SYNT is in the form
*	`./DES_Supervisor_linux <options...>`
*	`./DES_Supervisor_win <options...>`
	
### Options List:
* Mode `[-m]` - switch between interactive mode `[INTERACTIVE]`, the BSCOPNBMAX `[no arg]`, the MPO `[MPO]`, the MPRCP `[MPRCP]` or the converter `[convert]`
* MPO_condition `[-c]` - request the MPO to find a `[min]`imal or `[max]`imal solution
* FSM_file `[-f]` - provide an FSM file for processing
* Property `[-p]` - provide an implemented information state property
* ISP_file `[-i]` - provide a corresponding file for the specified ISP property
* Required_behavior `[-r]` - provide an required_behavior FSM file for MPRCP mode
* Verbose `[-v]` - request more detailed output
* Write_to_File `[-w]` - write all structures relevant to current mode to
							 separate .fsm files in the ./results folder
* Help `[-h]` - display help menu
                
### Examples:

		
* Compute supervisor for FSM_test_4.txt w/ safety property and no output
`./bin/DES_Supervisor -f ./test/FSM_test_4.txt -p safety -i ./test/safety_test_4.txt`
		
* Compute supervisor for FSM_test_6.txt w/ no property and both screen and file output
`./bin/DES_Supervisor -f ./test/FSM_test_6.txt -v -w`
		
* Compute MPO for FSM_test_2.txt
`./bin/DES_Supervisor -f ./test/FSM_test_2.txt -m MPO`
	  	
* Compute maximal MPO for FSM_test_25.fsm w/ state disambiguation property and full output
`./bin/DES_Supervisor   -f ./test/FSM_test_25.txt -v -w -p disambuguation -i ./test/disambuguation_test_25.txt -m mpo`
	
* Compute MPRCP supervisor for test_1.txt w/ the required behavior test_req1.txt, 
		the safety property test_safety1.txt and full output           
`./bin/DES_Supervisor  -m MPRCP -f ./test/test_1.txt -p safety -i ./test/test_safety1.txt -r ./test/test_req1.txt -v`
                
* Convert a .txt FSM file to a .fsm FSM file
`./bin/DES_Supervisor -f ./test/FSM_test_4.txt -m CONVERT`
		
* Start interactive mode
`./bin/Des_Supervisor -m interactive`
		
* NOTE: All input files should be in Unix format. If unexpected results
	occur, try running dos2unix on the input files.
	
## FSM_file and Format:

* The FSM_file describes the Finite State Machine that will be used to
construct the supervisor. Its format is as follows:

		<number_of_states> <number_of_events>

		<state1> <marked (m or u)>
		...
		<staten> <marked (m or u)>

		<event1> <controllability (c or u)> <observability (o or u)>
		...
		<eventn> <controllability (c or u)> <observability (o or u)>

		<event state> <event state> ... //state1 transitions
		...
		<event state> <event state> ... //staten transitions
See the FSM_test_.txt and test_ files in the ./test folder for examples.
For the Required behavior file, the format is the same as the format of the FSM_file. 
The events in the Required behavior file have to be the same as the events of the FSM_file. 
	
* ISP:
	The Information State Properties currently defined for this program
	are safety, opacity, and state disambiguation. See "Adding ISPs" for information on how to
	add additional ISPs. If this is left blank or does not match any ISP,
	the supervisor will be constructed with a trivial ISP.
	
* ISP_file:
	The ISP_file contains additional information specific to the ISP being
	used in the program. If this is left blank or does not match any file,
	the supervisor will be constructed with a trivial ISP.
		For safety, this file contains a list of the unsafe states.
		For opacity, this file contains a list of secret states.
		For state disambiguation, this file contains two lines of disjoint states
	See `./test/<ISP>_test_*.txt` for examples of valid ISP_files.
	NOTE: The test numbers of the FSM and ISP files in ./test correspond to
	one another.

* Adding ISPs:
	ISPs are hardcoded in files `./src/IS_Property.cpp` and
	`./include/IS_Property.h` as derived classes of class IS_Property. You may
	use the existing properties as a template to construct your own, or
	contact the programmer.
	NOTE: The function get_ISP in the mentioned files must also be modified
	in order to accept new command line arguments for the new ISP.
	
* Safety for MPRCP:
	The safety ISP_file for MPRCP is different with other modes. It contains a FSM instead
	of a list of the unsafe states. The format is the same as the format of the FSM_file.
	See `./test/test_safety*.txt` for examples of valid safety ISP_files for MPRCP.
	A set of test files for a single testcase for mode MPRCP is `./test/test_*.txt`, 
	`./test/test_req*.txt` and `./test/test_safety*.txt`. Also the correspondent output is `./test/test_*.out`.

* Interactive Mode:
	The default mode of the program is the interactive mode, if there are no arguments. 
	There are instructions shown in this mode. You can type in things according to the instructions.

##References:


[1] Yin, X. and Lafortune, S. (2016). Synthesis of maximally permissive supervisors for partially observed discrete event systems. 
    IEEE Transactions on Automatic Control, 61(5), 1239-1254.
    
[2]	Yin, X. and Lafortune, S. (2016). A uniform approach for synthesizing property-enforcing supervisors for partially-observed discrete-event systems.   IEEE Transactions on Automatic Control, 61(8), 2140-2154.	
    
[3] Yin, X. and Lafortune, S. (2015). A general approach for solving dynamic sensor activation problems for a class of properties. 
    In Proceedings of the 54th IEEE Conference on Decision and Control, 3610-3615.
    
[4] Yin, X. and Lafortune, S. (2016). Synthesis of maximally-permissive supervisors for the range control problem.  IEEE Transactions on Automatic Control. DOI: 10.1109/TAC.2016.2644867
