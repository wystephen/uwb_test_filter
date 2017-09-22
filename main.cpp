#include <iostream>
#include <fstream>
#include <fcntl.h>


#include "KalmanFilter.hpp"


int main() {

    std::ifstream if_data;
    if_data.open("test_data.txt");

    std::ofstream out_res("out_data.txt");

    double x,y;

    std::string value;
    own::KalmanFilter<double,4,2> kf_test(20.0,1.0,Eigen::Vector4d(0.5,0.5,2.5,2.5));

    bool first_time = true;

    while(if_data.good())
    {
        getline(if_data,value,' ');
        x = atof(value.c_str());
        getline(if_data,value,'\n');
        y = atof(value.c_str());
        std::cout << x<<","<< y << std::endl;
        if(first_time)
        {
            kf_test.InitialState(
                    Eigen::Vector4d(x,y,0.0,0.0)
            );
            first_time = false;
        }else{
            auto state = kf_test.OneStep(
                    Eigen::Vector2d(x,y)
            );
            out_res << state(0) << "," << state(1) << std::endl;
        }


    }

    return 0;
}