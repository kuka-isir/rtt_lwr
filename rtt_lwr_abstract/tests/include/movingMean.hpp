#ifndef RTE_MOVING_MEAN_HPP
#define RTE_MOVING_MEAN_HPP

#include <vector>

class MovingMean {
    public:
        MovingMean();
        MovingMean(unsigned int numObs);

        double getMean(double obs);
        void setNumObs(unsigned int numObs);

        unsigned int n;
        std::vector<double> vect_obs;
};

#endif
