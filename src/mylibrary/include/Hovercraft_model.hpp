//Hovercraft_model.hpp

#ifndef HOVERCRAFT_MODEL_H
#define HOVERCRAFT_MODEL_H

#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

class Hovercraft_model
{
	public:
		// constructor
		Hovercraft_model();

	private:
		// transition map
		Function f;

		// measurment map
		Function h;
};

#endif
