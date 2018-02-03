#include<casadi/casadi.hpp>
#include<iostream>

using namespace std;
using namespace casadi;

void foo(Function f, Function g)
{
	MX x = MX::sym("x",1);
	MX y = MX::sym("y",1);
	cout << f(MXVector{x}) << endl;
	cout << g(MXVector{x,y}) << endl;

	Function func2 = Function("func2", {x,y}, {f(MXVector{x})[0], g(MXVector{x,y})[0]});
}

int main()
{
//	MX x = MX::sym("x", 1);
//	MX y = MX::sym("y", 1);
//	MX z = MX::sym("z", 1);
//	MX f = pow(x,2) + 100 * pow(z,2);
//	MX g = z + pow((1-x), 2) - y;
//	MXDict nlp = {{"f", f}, {"g", g}, {"x", MX::vertcat({x,y,z})}};
//	Dict opts = {};
//	//opts = {{"ipopt.linear_solver", "ma97"}};
//	Function solver = nlpsol("S","ipopt",nlp,opts);
//	DMDict arg ={{"ubg", 0}, {"lbg", 0}, {"x0", DM::vertcat({0,0,0})}};
//	DMDict res = solver(arg);
//	cout << "hello alic" << endl;
//
//	// some test function 
//	Function alic = Function("alic",{x,y},{x+y},{"var1","var2"},{"sortie"});
//	DMDict input = {{"var1", 1},{"var2", 2}};
//	DMDict output = alic(input);
//	cout << "la réponse vaut : " << output["sortie"] << endl;
//
//	Function func = Function("func", {x}, {pow(x,2)});
	MX x = MX::sym("x",1);
	MX y = MX::sym("y",1);
	Function f = Function("f", {x}, {pow(x,2)});
	Function g = Function("g", {x,y}, {x+y});
	foo(f,g);
	return 0;
}
//
