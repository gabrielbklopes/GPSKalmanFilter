# GPSKalmanFilter

The functioning of this library is very simple. For the sack of my sanity I didnt't pass the matrices as a reference, which can be later changed in the code.

In order to make it work define the matrices x_N, x_E, P_N, P_E. These are the position and covariance for each axis in the measurement. Call the begin function for the library and pass it as parameters, together with the time dt that represents the frequency of update of the filter. This value has to match with the code running time.

You can then, call the filterCalculation function and pass the values to the filter.  Previous Latitude, Previous Longitude, Current Latitude, Current Longitude, Velocity, acceleration, heading angle, a pointer to the place storing the latitude and longitude calculated by the filter.

Remember this library doesn't implement the best code practices but it works just fine.

If any questions regarding this development contact me on my linkedin: https://www.linkedin.com/in/gabrielbklopes/ or in my email: gabrielbklopes@gmail.com

Gabriel Lopes!
