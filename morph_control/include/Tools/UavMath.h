#pragma once

#include <cmath>
#include <Eigen/Eigen>

#define gravity_constant 9.816
#define _min(a,b) (a<b ? a:b)
#define _max(a,b) (a>b ? a:b)
#define _constrain(val,min_val,max_val) (_min(_max(val,min_val),max_val))
//template <typename T>
//inline T _constrain(T val, T min_val, T max_val) { return _min(_max(val,min_val),max_val); }

template <typename T>
inline T _sign(T val);

// degree and rad transformation
template <typename T>
inline T deg2rad(T degree);
template <typename T>
inline T rad2deg(T radians);

// Kahan summation algorithm to get the sum value
template <typename T>
inline T kahanSummation(T sum_previous, T input, T& accumulator);

// Maximum and minimum functions
template <typename T>
Eigen::Matrix<T,2,1> max_2d(Eigen::Matrix<T,2,1> x, T val); 
template <typename T>
Eigen::Matrix<T,2,1> min_2d(Eigen::Matrix<T,2,1> x, T val); 
template <typename T>
Eigen::Matrix<T,2,1> constrain_2d(Eigen::Matrix<T,2,1> x, T min_val, T max_val); 
template <typename T>
Eigen::Matrix<T,3,1> max_3d(Eigen::Matrix<T,3,1> x, T val); 
template <typename T>
Eigen::Matrix<T,3,1> min_3d(Eigen::Matrix<T,3,1> x, T val); 
template <typename T>
Eigen::Matrix<T,3,1> constrain_3d(Eigen::Matrix<T,3,1> x, T min_val, T max_val); 
template <typename T>
Eigen::Matrix<T,4,1> max_4d(Eigen::Matrix<T,4,1> x, T val); 
template <typename T>
Eigen::Matrix<T,4,1> min_4d(Eigen::Matrix<T,4,1> x, T val); 
template <typename T>
Eigen::Matrix<T,4,1> constrain_4d(Eigen::Matrix<T,4,1> x, T min_val, T max_val); 

// Saturation functions
template <typename T>
Eigen::Matrix<T,2,1> sat_2d(Eigen::Matrix<T,2,1> x, T max_val); 
template <typename T>
Eigen::Matrix<T,2,1> minus_sat_2d(Eigen::Matrix<T,2,1> x, T min_val); 
template <typename T>
Eigen::Matrix<T,3,1> sat_3d(Eigen::Matrix<T,3,1> x, T max_val); 
template <typename T>
Eigen::Matrix<T,3,1> minus_sat_3d(Eigen::Matrix<T,3,1> x, T min_val); 
template <typename T>
Eigen::Matrix<T,4,1> sat_4d(Eigen::Matrix<T,4,1> x, T max_val); 
template <typename T>
Eigen::Matrix<T,4,1> minus_sat_4d(Eigen::Matrix<T,4,1> x, T min_val); 

template <typename T>
Eigen::Matrix<T,2,1> sat_gd_2d(Eigen::Matrix<T,2,1> x, T max_val);  // preserve vector direction
template <typename T>
Eigen::Matrix<T,2,1> minus_sat_gd_2d(Eigen::Matrix<T,2,1> x, T min_val);
template <typename T>
Eigen::Matrix<T,3,1> sat_gd_3d(Eigen::Matrix<T,3,1> x, T max_val); 
template <typename T>
Eigen::Matrix<T,3,1> minus_sat_gd_3d(Eigen::Matrix<T,3,1> x, T min_val);
template <typename T>
Eigen::Matrix<T,4,1> sat_gd_4d(Eigen::Matrix<T,4,1> x, T max_val); 
template <typename T>
Eigen::Matrix<T,4,1> minus_sat_gd_4d(Eigen::Matrix<T,4,1> x, T min_val);


// Create Eigen vector functions
// Create 2d vector
template <typename T>
Eigen::Matrix<T,2,1> create_vector_2d(T x, T y);
template <typename T>
Eigen::Matrix<T,2,1> create_vector_2d(T* input);

// Create 3d vector
template <typename T>
Eigen::Matrix<T,3,1> create_vector_3d(T x, T y, T z);
template <typename T>
Eigen::Matrix<T,3,1> create_vector_3d(T* input);

// Create 4d vector
template <typename T>
Eigen::Matrix<T,4,1> create_vector_4d(T w, T x, T y, T z);
template <typename T>
Eigen::Matrix<T,4,1> create_vector_4d(T* input);


// Attitude correlation functions
// rpy: Euler angle;  q: Quaternion;  R: Rotation matrix
// Euler angle and Rotation matrix is defined in order ZYX
// Quaternion is defined in order WXYZ
// i represent inertial coordinate (world coordinate)
// b represent uav body coordinate

// Calculate Rotation matrix from body to ground R^{i}_{b}
template <typename T>
Eigen::Matrix<T,3,3> rpy2R(T r, T p, T y);
template <typename T>
Eigen::Matrix<T,3,3> rpy2R(Eigen::Matrix<T,3,1> rpy);

// Calculate Euler angle from Rotation matrix
template <typename T>
Eigen::Matrix<T,3,1> R2rpy(Eigen::Matrix<T,3,3> R);

// Calculate Quaternion from Eular angle
template <typename T>
Eigen::Matrix<T,4,1> rpy2q(T r, T p, T y);
template <typename T>
Eigen::Matrix<T,4,1> rpy2q(Eigen::Matrix<T,3,1> rpy);
// Casually add a parameter to return Eigen::Quaternion
template <typename T>
Eigen::Quaternion<T> rpy2q(T r, T p, T y, uint8_t rand);
template <typename T>
Eigen::Quaternion<T> rpy2q(Eigen::Matrix<T,3,1> rpy, uint8_t rand);

// Calculate Euler angle from Quaternion
template <typename T>
Eigen::Matrix<T,3,1> q2rpy(T w, T x, T y, T z);
template <typename T>
Eigen::Matrix<T,3,1> q2rpy(Eigen::Matrix<T,4,1> q);
template <typename T>
Eigen::Matrix<T,3,1> q2rpy(Eigen::Quaternion<T> q);

// Calculate Rotation matrix from Quaternion
template <typename T>
Eigen::Matrix<T,3,3> q2R(T w, T x, T y, T z);
template <typename T>
Eigen::Matrix<T,3,3> q2R(Eigen::Matrix<T,4,1> q);
template <typename T>
Eigen::Matrix<T,3,3> q2R(Eigen::Quaternion<T> q);

// Calculate Rotate angle vector from Quaternion
template <typename T>
Eigen::Matrix<double,3,1> q2rot(Eigen::Quaternion<T> q);

// Calculate Rotation matrix in one channel
// R^{i}_{b} = R^{T}_z(psi) * R^{T}_y(theta) * R^{T}_x(phi)
template <typename T>
Eigen::Matrix<T,3,3> compute_R(T r, T p, T y);  // tow value in r, p, y should be zero

// Calculate mapping matrix between Euler angle change rate and angular velocity (described in body coordinates)
//  w^{b}_{b} = Q * \dot{Phi}
template <typename T>
Eigen::Matrix<T,3,3> compute_Q(T r, T p, T y);
template <typename T>
Eigen::Matrix<T,3,3> compute_Q(Eigen::Matrix<T,3,1> rpy);

// Calculate mapping matrix between angular velocity (described in body coordinates) and Euler angle change rate
// \dot{Phi} = W * w^{b}_{b}
template <typename T>
Eigen::Matrix<T,3,3> compute_W(T r, T p, T y);
template <typename T>
Eigen::Matrix<T,3,3> compute_W(Eigen::Matrix<T,3,1> rpy);

// Calculate euler angle acceleration from euler angle, angular velocity and angular acceleration
template <typename T>
Eigen::Matrix<T,3,1> compute_rpy_acc(Eigen::Matrix<T,3,1> rpy, Eigen::Matrix<T,3,1> w, Eigen::Matrix<T,3,1> ang_acc);
template <typename T>
Eigen::Matrix<T,3,1> compute_rpy_acc(Eigen::Matrix<T,3,1> rpy, Eigen::Matrix<T,3,1> rpy_speed, Eigen::Matrix<T,3,1> w, Eigen::Matrix<T,3,1> ang_acc);

// Calculate skew matrix from a 3d vector (subscript ^)
template <typename T>
Eigen::Matrix<T,3,3> skew(Eigen::Matrix<T,3,1> input);

// Calculate Quaternion Omega matrix (used in Quaternion multiplication)
// q * p = (qw*Identity + Omega(q)) * p
template <typename T>
Eigen::Matrix<T,4,4> compute_Omega(Eigen::Matrix<T,4,1> q);
template <typename T>
Eigen::Matrix<T,4,4> compute_Omega(Eigen::Quaternion<T> q);

// Calculate Quaternion multiplication
template <typename T>
Eigen::Matrix<T,4,1> mul_q(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2);


template <typename T>
inline T _sign(T val)
{
    if (val > 0) return 1;
    if (val < 0) return -1;
    return 0;
}

template <typename T>
inline T deg2rad(T degree)
{
    return degree * M_PI / 180;
}

template <typename T>
inline T rad2deg(T radians)
{
    return radians * 180 / M_PI;
}

template <typename T>
inline T kahanSummation(T sum_previous, T input, T& accumulator)
{
    const T y = input - accumulator;
    const T t = sum_previous + y;
    accumulator = (t - sum_previous) - y;
    return t;
}

template <typename T>
Eigen::Matrix<T,2,1> max_2d(Eigen::Matrix<T,2,1> x, T val)
{
    for (uint8_t i=0; i<2; ++i)
        if (x(i,0) < val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> min_2d(Eigen::Matrix<T,2,1> x, T val)
{
    for (uint8_t i=0; i<2; ++i)
        if (x(i,0) > val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> constrain_2d(Eigen::Matrix<T,2,1> x, T min_val, T max_val)
{
    x = max_2d(x, min_val);
    x = min_2d(x, max_val);
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> max_3d(Eigen::Matrix<T,3,1> x, T val)
{
    for (uint8_t i=0; i<3; ++i)
        if (x(i,0) < val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> min_3d(Eigen::Matrix<T,3,1> x, T val)
{
    for (uint8_t i=0; i<3; ++i)
        if (x(i,0) > val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> constrain_3d(Eigen::Matrix<T,3,1> x, T min_val, T max_val)
{
    x = max_3d(x, min_val);
    x = min_3d(x, max_val);
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> max_4d(Eigen::Matrix<T,4,1> x, T val)
{
    for (uint8_t i=0; i<4; ++i)
        if (x(i,0) < val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> min_4d(Eigen::Matrix<T,4,1> x, T val)
{
    for (uint8_t i=0; i<4; ++i)
        if (x(i,0) > val) x(i,0) = val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> constrain_4d(Eigen::Matrix<T,4,1> x, T min_val, T max_val)
{
    x = max_4d(x, min_val);
    x = min_4d(x, max_val);
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> sat_2d(Eigen::Matrix<T,2,1> x, T max_val)
{
    for (uint8_t i=0; i<2; ++i)
        if (x(i,0) > max_val) x(i,0) = max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> minus_sat_2d(Eigen::Matrix<T,2,1> x, T min_val)
{
    for (uint8_t i=0; i<2; ++i)
        if (x(i,0) < min_val) x(i,0) = min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> sat_3d(Eigen::Matrix<T,3,1> x, T max_val)
{
    for (uint8_t i=0; i<3; ++i)
        if (x(i,0) > max_val) x(i,0) = max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> minus_sat_3d(Eigen::Matrix<T,3,1> x, T min_val)
{
    for (uint8_t i=0; i<3; ++i)
        if (x(i,0) < min_val) x(i,0) = min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> sat_4d(Eigen::Matrix<T,4,1> x, T max_val)
{
    for (uint8_t i=0; i<4; ++i)
        if (x(i,0) > max_val) x(i,0) = max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> minus_sat_4d(Eigen::Matrix<T,4,1> x, T min_val)
{
    for (uint8_t i=0; i<4; ++i)
        if (x(i,0) < min_val) x(i,0) = min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> sat_gd_2d(Eigen::Matrix<T,2,1> x, T max_val)
{
    double real_max_val = x.maxCoeff();
    if (real_max_val < max_val) return x;
    x = x * max_val / real_max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,2,1> minus_sat_gd_2d(Eigen::Matrix<T,2,1> x, T min_val)
{
    double real_min_val = x.minCoeff();
    if (real_min_val > min_val) return x;
    x = x * min_val / real_min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> sat_gd_3d(Eigen::Matrix<T,3,1> x, T max_val)
{
    double real_max_val = x.maxCoeff();
    if (real_max_val < max_val) return x;
    x = x * max_val / real_max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,3,1> minus_sat_gd_3d(Eigen::Matrix<T,3,1> x, T min_val)
{
    double real_min_val = x.minCoeff();
    if (real_min_val > min_val) return x;
    x = x * min_val / real_min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> sat_gd_4d(Eigen::Matrix<T,4,1> x, T max_val)
{
    double real_max_val = x.maxCoeff();
    if (real_max_val < max_val) return x;
    x = x * max_val / real_max_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> minus_sat_gd_4d(Eigen::Matrix<T,4,1> x, T min_val)
{
    double real_min_val = x.minCoeff();
    if (real_min_val > min_val) return x;
    x = x * min_val / real_min_val;
    return x;
}

template <typename T>
Eigen::Matrix<T,4,1> create_vector_4d(T x, T y)
{
    Eigen::Matrix<T,4,1> output;
    output << x, y;
    return output;
}
template <typename T>
Eigen::Matrix<T,2,1> create_vector_2d(T* input)
{
    Eigen::Matrix<T,2,1> output;
    output << input[0], input[1];
    return output;
}

template <typename T>
Eigen::Matrix<T,3,1> create_vector_3d(T x, T y, T z)
{
    Eigen::Matrix<T,3,1> output;
    output << x, y, z;
    return output;
}
template <typename T>
Eigen::Matrix<T,3,1> create_vector_3d(T* input)
{
    Eigen::Matrix<T,3,1> output;
    output << input[0], input[1], input[2];
    return output;
}

template <typename T>
Eigen::Matrix<T,4,1> create_vector_4d(T w, T x, T y, T z)
{
    Eigen::Matrix<T,4,1> output;
    output << w, x, y, z;
    return output;
}
template <typename T>
Eigen::Matrix<T,4,1> create_vector_4d(T* input)
{
    Eigen::Matrix<T,4,1> output;
    output << input[0], input[1], input[2], input[3];
    return output;
}

template <typename T>
Eigen::Matrix<T,3,3> rpy2R(T r, T p, T y)
{
    Eigen::Matrix<T,3,3> output;
    output << cos(p)*cos(y), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r),
              cos(p)*sin(y), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r),
              -sin(p), sin(r)*cos(p), cos(r)*cos(p);
    return output; 
}
template <typename T>
Eigen::Matrix<T,3,3> rpy2R(Eigen::Matrix<T,3,1> rpy)
{
    Eigen::Matrix<T,3,3> output;
    T r=rpy(0,0), p=rpy(1,0), y=rpy(2,0);
    output << cos(p)*cos(y), cos(y)*sin(p)*sin(r)-sin(y)*cos(r), cos(y)*sin(p)*cos(r)+sin(y)*sin(r),
              cos(p)*sin(y), sin(y)*sin(p)*sin(r)+cos(y)*cos(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r),
              -sin(p), sin(r)*cos(p), cos(r)*cos(p);
    return output; 
}

template <typename T>
Eigen::Matrix<T,3,1> R2rpy(Eigen::Matrix<T,3,3> R)
{
    Eigen::Matrix<T,3,1> rpy;
    rpy << atan2(R(2,1), R(2,2)),
    	   atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2))),
           atan2(R(1,0), R(0,0));
    return rpy;
}

template <typename T>
Eigen::Matrix<T,4,1> rpy2q(T r, T p, T y)
{
    Eigen::Matrix<T,4,1> q;
    T hr=r/2, hp=p/2, hy=y/2;
    q << cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy),
         sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy),
         cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy),
         cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy);
    return q;
}
template <typename T>
Eigen::Matrix<T,4,1> rpy2q(Eigen::Matrix<T,3,1> rpy)
{
    Eigen::Matrix<T,4,1> q;
    T hr=rpy(0,0)/2, hp=rpy(1,0)/2, hy=rpy(2,0)/2;
    q << cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy),
         sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy),
         cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy),
         cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy);
    return q;
}
template <typename T>
Eigen::Quaternion<T> rpy2q(T r, T p, T y, uint8_t rand)
{
    Eigen::Quaternion<T> q;
    T hr=r/2, hp=p/2, hy=y/2;
    q.w() = cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy);
    q.x() = sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy);
    q.y() = cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy);
    q.z() = cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy);
    return q;
}
template <typename T>
Eigen::Quaternion<T> rpy2q(Eigen::Matrix<T,3,1> rpy, uint8_t rand)
{
    Eigen::Quaternion<T> q;
    T hr=rpy(0,0)/2, hp=rpy(1,0)/2, hy=rpy(2,0)/2;
    q.w() = cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy);
    q.x() = sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy);
    q.y() = cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy);
    q.z() = cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy);
    return q;
}

template <typename T>
Eigen::Matrix<T,3,1> q2rpy(T w, T x, T y, T z)
{
    Eigen::Matrix<T,3,1> rpy;
    rpy << atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
           asin(2*(w*y-x*z)),
           atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return rpy;
}
template <typename T>
Eigen::Matrix<T,3,1> q2rpy(Eigen::Matrix<T,4,1> q)
{
    Eigen::Matrix<T,3,1> rpy;
    T w=q(0,0), x=q(1,0), y=q(2,0), z=q(3,0);
    rpy << atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
           asin(2*(w*y-x*z)),
           atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return rpy;
}
template <typename T>
Eigen::Matrix<T,3,1> q2rpy(Eigen::Quaternion<T> q)
{
    Eigen::Matrix<T,3,1> rpy;
    T w=q.w(), x=q.x(), y=q.y(), z=q.z();
    rpy << atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
           asin(2*(w*y-x*z)),
           atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return rpy;
}

template <typename T>
Eigen::Matrix<T,3,3> q2R(T w, T x, T y, T z)
{
    Eigen::Matrix<T,3,3> output;
    output << 1-2*y*y-2*z*z, 2*(x*y-w*z), 2*(x*z+w*y),
              2*(x*y+w*z), 1-2*x*x-2*z*z, 2*(y*z-w*x),
              2*(x*z-w*y), 2*(y*z+w*x), 1-2*x*x-2*y*y;
    return output;
}
template <typename T>
Eigen::Matrix<T,3,3> q2R(Eigen::Matrix<T,4,1> q)
{
    Eigen::Matrix<T,3,3> output;
    T w=q(0,0), x=q(1,0), y=q(2,0), z=q(3,0);
    output << 1-2*y*y-2*z*z, 2*(x*y-w*z), 2*(x*z+w*y),
              2*(x*y+w*z), 1-2*x*x-2*z*z, 2*(y*z-w*x),
              2*(x*z-w*y), 2*(y*z+w*x), 1-2*x*x-2*y*y;
    return output;
}
template <typename T>
Eigen::Matrix<T,3,3> q2R(Eigen::Quaternion<T> q)
{
    Eigen::Matrix<T,3,3> output;
    T w=q.w(), x=q.x(), y=q.y(), z=q.z();
    output << 1-2*y*y-2*z*z, 2*(x*y-w*z), 2*(x*z+w*y),
              2*(x*y+w*z), 1-2*x*x-2*z*z, 2*(y*z-w*x),
              2*(x*z-w*y), 2*(y*z+w*x), 1-2*x*x-2*y*y;
    return output;
}

template <typename T>
Eigen::Matrix<double,3,1> q2rot(Eigen::Quaternion<T> q)
{
    Eigen::AngleAxisd rot(q);
    Eigen::Matrix<double,3,1> rot_vec = rot.angle() * rot.axis();
    return rot_vec;
}

template <typename T>
Eigen::Matrix<T,3,3> compute_R(T r, T p, T y)
{
    Eigen::Matrix<T,3,3> output;
    if(!r)
        output << 1, 0, 0,
                  0,cos(r),-sin(r),
                  0,sin(r), cos(r);
    else if(!p)
        output << cos(p),0,sin(p),
                  0, 1, 0,
                  -sin(p),0,cos(p);
    else
        output << cos(y),-sin(y),0,
                  sin(y),cos(y),0,
                  0, 0, 1;
    return output;
}

template <typename T>
Eigen::Matrix<T,3,3> compute_Q(T r, T p, T y)
{
    Eigen::Matrix<T,3,3> Q;
    Q << 1, 0, -sin(p),
         0, cos(r), cos(p)*sin(r),
         0, -sin(r), cos(p)*cos(r);
    return Q;
}

template <typename T>
Eigen::Matrix<T,3,3> compute_Q(Eigen::Matrix<T,3,1> rpy)
{
    Eigen::Matrix<T,3,3> Q;
    T r=rpy(0,0), p=rpy(1,0);
    Q << 1, 0, -sin(p),
         0, cos(r), cos(p)*sin(r),
         0, -sin(r), cos(p)*cos(r);
    return Q;
}

template <typename T>
Eigen::Matrix<T,3,3> compute_W(T r, T p, T y)
{
    Eigen::Matrix<T,3,3> W;
    W << 1, tan(p)*sin(r), tan(p)*cos(r),
         0, cos(r), -sin(r),
         0, sin(r)/cos(p), cos(r)/cos(p);
    return W;
}
template <typename T>
Eigen::Matrix<T,3,3> compute_W(Eigen::Matrix<T,3,1> rpy)
{
    Eigen::Matrix<T,3,3> W;
    T r=rpy(0,0), p=rpy(1,0);
    W << 1, tan(p)*sin(r), tan(p)*cos(r),
         0, cos(r), -sin(r),
         0, sin(r)/cos(p), cos(r)/cos(p);
    return W;
}

template <typename T>
Eigen::Matrix<T,3,1> compute_rpy_acc(Eigen::Matrix<T,3,1> rpy, Eigen::Matrix<T,3,1> w, Eigen::Matrix<T,3,1> ang_acc)
{
    Eigen::Matrix<T,3,1> rpy_speed, rpy_acc;
    rpy_speed.noalias() = compute_W(rpy) * w;
    T r = rpy(0,0), p = rpy(1,0), y = rpy(2,0);
    T r_diff = rpy_speed(0,0), p_diff = rpy_speed(1,0), y_diff = rpy_speed(2,0);

    rpy_acc << ang_acc(0,0) + sin(r)*w(1,0)*p_diff/pow(cos(p),2) + tan(p)*cos(r)*w(1,0)*r_diff + tan(p)*sin(r)*ang_acc(1,0)
               + cos(r)*w(2,0)*p_diff/pow(cos(p),2) - tan(p)*sin(r)*w(2,0)*r_diff + tan(p)*cos(r)*ang_acc(2,0),
               -sin(r)*w(1,0)*r_diff + cos(r)*ang_acc(1,0) - cos(r)*w(2,0)*r_diff - sin(r)*ang_acc(2,0), 
               (cos(r)*cos(p)*r_diff+sin(r)*sin(p)*p_diff)*w(1,0)/pow(cos(p),2) + sin(r)*ang_acc(1,0)/cos(p)
               + (-sin(r)*cos(p)*r_diff+cos(r)*sin(p)*p_diff)*w(2,0)/pow(cos(p),2) + cos(r)*ang_acc(2,0)/cos(p);
    return rpy_acc;
}

template <typename T>
Eigen::Matrix<T,3,1> compute_rpy_acc(Eigen::Matrix<T,3,1> rpy, Eigen::Matrix<T,3,1> rpy_speed, Eigen::Matrix<T,3,1> w, Eigen::Matrix<T,3,1> ang_acc)
{
    Eigen::Matrix<T,3,1>  rpy_acc;
    T r = rpy(0,0), p = rpy(1,0), y = rpy(2,0);
    T r_diff = rpy_speed(0,0), p_diff = rpy_speed(1,0), y_diff = rpy_speed(2,0);

    rpy_acc << ang_acc(0,0) + sin(r)*w(1,0)*p_diff/pow(cos(p),2) + tan(p)*cos(r)*w(1,0)*r_diff + tan(p)*sin(r)*ang_acc(1,0)
               + cos(r)*w(2,0)*p_diff/pow(cos(p),2) - tan(p)*sin(r)*w(2,0)*r_diff + tan(p)*cos(r)*ang_acc(2,0),
               -sin(r)*w(1,0)*r_diff + cos(r)*ang_acc(1,0) - cos(r)*w(2,0)*r_diff - sin(r)*ang_acc(2,0), 
               (cos(r)*cos(p)*r_diff+sin(r)*sin(p)*p_diff)*w(1,0)/pow(cos(p),2) + sin(r)*ang_acc(1,0)/cos(p)
               + (-sin(r)*cos(p)*r_diff+cos(r)*sin(p)*p_diff)*w(2,0)/pow(cos(p),2) + cos(r)*ang_acc(2,0)/cos(p);
    return rpy_acc;
}

template <typename T>
Eigen::Matrix<T,3,3> skew(Eigen::Matrix<T,3,1> input)
{
    Eigen::Matrix<T,3,3> output;
    output << 0, -input(2,0), input(1,0),
              input(2,0), 0, -input(0,0),
              -input(1,0), input(0,0), 0;
    return output;
}

template <typename T>
Eigen::Matrix<T,4,4> compute_Omega(Eigen::Matrix<T,4,1> q)
{
    Eigen::Matrix<T,4,4> Omega;
    Omega << 0, -q(1,0), -q(2,0), -q(3,0),
                          q(1,0), 0, q(3,0), -q(2,0),
                          q(2,0), -q(3,0), 0, q(1,0),
                          q(3,0), q(2,0), -q(1,0), 0;
    return Omega;
}

template <typename T>
Eigen::Matrix<T,4,4> compute_Omega(Eigen::Quaternion<T> q)
{
    Eigen::Matrix<T,4,4> Omega;
    Omega << 0, -q.x(), -q.y(), -q.z(),
                          q.x(), 0, q.z(), -q.y(),
                          q.y(), -q.z(), 0, q.x(),
                          q.z(), q.y(), -q.x(), 0;
    return Omega;
}

template <typename T>
Eigen::Matrix<T,4,1> mul_q(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2)
{
    T qw=q1(0,0), qx=q1(1,0), qy=q1(2,0), qz=q1(3,0);
    Eigen::Matrix<T,4,4> tmp;
    tmp << qw, -qx, -qy, -qz,
           qx, qw, -qz, qy,
           qy, qz, qw, -qx,
           qz, -qy, qx, qw;
    return tmp*q2;
}
