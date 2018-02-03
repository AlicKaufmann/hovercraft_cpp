#include<Reference.hpp>


Reference::Reference()
{
	//create the reference function
	int r = 2;
	MX t = MX::sym("t",1);
	MX rhs = MX::vertcat({r*cos(0.5 * t), r * sin(0.5 * t)});
}
