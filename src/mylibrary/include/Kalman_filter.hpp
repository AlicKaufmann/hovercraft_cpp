//kalman_filter.hpp

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

class Kalman_filter
{
        public:
		// constructor
                Kalman_filter(Function fd, Function h, MX Q, MX R);
        private:
                MX Q;
                MX R;
                Function predict;
                //Function update;

};

#endif
