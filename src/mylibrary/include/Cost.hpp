// Cost.hpp

#ifndef COST_H
#define COST_H

#include <casadi/casadi.hpp>
#include <Reference.hpp>

using namespace std;
using namespace casadi;

class Cost
{
	public:
		// constructor
		Cost(Reference ref);

		Function L;
	private:
};
#endif
