#include <movingMean.hpp>
#include <algorithm>

MovingMean::MovingMean(){
    n = 0;
}

MovingMean::MovingMean(unsigned int numObs){
    setNumObs(numObs);
}

void MovingMean::setNumObs(unsigned int numObs){
    if( n < numObs ){
        int size = vect_obs.size();
        for(unsigned int i = 0; i<numObs-size; i++){
            vect_obs.push_back(0.0);
        }
    }
    else{
        vect_obs.resize(numObs);
    }
    n = numObs;
}

double MovingMean::getMean(double obs){
    double mean = 0.0;
    if(n == 0){
        mean = 0.0;
    }
    else{
        std::rotate(vect_obs.begin(), vect_obs.begin()+1, vect_obs.end());
        vect_obs.back() = obs;

        for(std::vector<double>::iterator iter = vect_obs.begin(); iter != vect_obs.end(); ++iter){
            mean += *iter/n;
        }
    }

    return mean;
}
