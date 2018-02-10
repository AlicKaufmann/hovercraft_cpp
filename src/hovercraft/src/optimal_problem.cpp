#include<casadi/casadi.hpp>
#include<iostream>

using namespace std;
using namespace casadi;



int main()
{	
	vector<DM> v(3);
	v[0] = DM::vertcat({0,1,2});
	v[1] = DM::vertcat({3,4,5});
	v[2] = DM::vertcat({6,7,8});
	DM vec = DM::vertcat(v); 
	cout << vec << endl;	
	cout << vec.size() << endl;
	return 0;
}
//
