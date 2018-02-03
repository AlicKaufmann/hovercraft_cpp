#ifndef REFERENCE_H
#define REFERENCE_H

#include<casadi/casadi.hpp>

using namespace std;
using namespace casadi;

class Reference
{
	friend class Cost;

	// constructor
	public:
		Reference();
	private:
		Function ref;		

};
#endif

