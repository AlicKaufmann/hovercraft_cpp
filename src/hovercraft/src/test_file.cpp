#include <casadi/casadi.hpp>
//using std::cout;
using namespace std;
using namespace casadi;
                                                                              
int main()                                                                      
{                                                                              
        MX x = MX::sym("x", 1);                                                
        MX y = MX::sym("y", 1);                                                
        MX z = MX::sym("z", 1);                                                
        MX f = pow(x,2) + 100 * pow(z,2);                                      
        MX g = z + pow((1-x), 2) - y;                                          
        MXDict nlp = {{"f", f}, {"g", g}, {"x", MX::vertcat({x,y,z})}};        
	//Dict opts ={};
	Dict opts = {{"ipopt.linear_solver", "ma97"}};
        Function solver = nlpsol("S","ipopt", nlp, opts);             
        DMDict arg ={{"ubg", 0}, {"lbg", 0}, {"x0", DM::vertcat({0,0,0})}}; 
	DMDict res = solver(arg);
	vector<double> uopt(res["x"]);
	cout << " la solution optimal est : " << uopt << endl;
	return 0;                                                              

	//std::vector<casadi::MX> U(3);
	//// this happens in the loop!
	//U[0] = casadi::MX::sym("U1",2);
	//U[1] = casadi::MX::sym("U2",2);
	//U[2] = casadi::MX::sym("U3",3);
	////cout << "hello world" << std::endl;
	//cout << U << std::endl;
	//cout << casadi::MX::vertcat(U) << std::endl;
	//vector<MX> u ={};
	//for(int k=0; k<=5; k++)
	//{
	//	MX uk = MX::sym("u" + to_string(k), 2);
	//	u.push_back(uk);
	//	cout << u << endl;
	//}
	//cout << MX::vertcat(u) << endl;
	//MX xk = MX::sym("xk", 10);
	//cout << "la taille de xk est : " << xk.size() << endl;
	//cout << "xk(0,0) : " << xk(0,0) << endl;
	//MX sk = xk.nz(Slice(5,8,1));
	//cout << sk << endl;
	//cout << "la taille du slice est " << sk.size() << endl;
	//cout << "olééé " << sk(0) << endl;
	//cout << "olééé " << sk(1) << endl;
	//cout << "olééé " << sk(2) << endl;
	////cout << "olééé " << sk(3) << endl;
}
