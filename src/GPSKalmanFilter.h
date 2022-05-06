/* 
 * GPSKalmanFilter - a Kalman Filter implementation for GPS .
 * Created by Gabriel Lopes, March, 24, 2022.
 * 
 */

#ifndef GPSKFilter
#define GPSKFilter

#include "Haversine.h"
#include "Arduino.h"



class GPSKF
{
    public:
        GPSKF(double dt, double N, double E, double x_N[2][1], double P_N[2][2], double x_E[2][1], double P_E[2][2]);
        void filterCalculation(double lat_old, double lng_old, double lat_cur, double lng_cur, double velocity, double acceleration, double &latitude_filter, double &longitude_filter); 
    
    private:
        double _dt;
        double N;
        double E;
        //x -> state vector
        double _x_N[2][1]; //= {{0},{0}};
        //P -> Covariance Matrix
        double _P_N[2][2]; //= {{100, 0},{0, 100}};
        //x -> state vector
        double _x_E[2][1]; //= {{0},{0}};
        //P -> Covariance Matrix
        double _P_E[2][2]; //= {{100, 0},{0, 100}};
        
};
#endif
