//Hovercraft_model.hpp

#ifndef HOVERCRAFT_MODEL_H
#define HOVERCRAFT_MODEL_H

#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

class Hovercraft_model
{
	friend class MPC;
	public:
		// constructor
		Hovercraft_model();

		// transition map
		Function f;

		// measurment map
		Function h;
	private:
};

#endif
