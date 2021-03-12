/*
*) Refactor the code to remove reduentent part. 
*) Compiled for Linux with C++14 standard
Copyright (c) 2021 Balamurugan Kandan.
MIT License; See LICENSE.md for complete details
Author: 2021 Balamurugan Kandan
*/

/*
Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to get gyro and accel bias. Added initialization to
estimated angles rather than assuming IMU is level.

Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

/*
Addapted from earlier version
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie
*/

#include "ekfNavINS.h"

void ekfNavINS::ekf_init(uint64_t time, double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz) {
  // grab initial gyro values for biases
  gbx = p;
  gby = q;
  gbz = r;
  std::tie(theta,phi,psi) = getPitchRollYaw(ax, ay, az, hx, hy, hz);
  // euler to quaternion
  quat = toQuaternion(phi, theta, psi);
  // Assemble the matrices
  // ... gravity
  grav(2,0) = G;
  // ... H
  H.block(0,0,5,5) = Eigen::Matrix<float,5,5>::Identity();
  // ... Rw
  Rw.block(0,0,3,3) = powf(SIG_W_A,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(3,3,3,3) = powf(SIG_W_G,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  Rw.block(6,6,3,3) = 2.0f * powf(SIG_A_D,2.0f) / TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(9,9,3,3) = 2.0f * powf(SIG_G_D,2.0f) / TAU_G*Eigen::Matrix<float,3,3>::Identity();
  // ... P
  P.block(0,0,3,3) = powf(P_P_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(3,3,3,3) = powf(P_V_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(6,6,2,2) = powf(P_A_INIT,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  P(8,8) = powf(P_HDG_INIT,2.0f);
  P.block(9,9,3,3) = powf(P_AB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(12,12,3,3) = powf(P_GB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  // ... R
  R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(2,2) = powf(SIG_GPS_P_D,2.0f);
  R.block(3,3,2,2) = powf(SIG_GPS_V_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(5,5) = powf(SIG_GPS_V_D,2.0f);
  // .. then initialize states with GPS Data
  lat_ins = lat;
  lon_ins = lon;
  alt_ins = alt;
  vn_ins = vn;
  ve_ins = ve;
  vd_ins = vd;
  // specific force
  f_b(0,0) = ax;
  f_b(1,0) = ay;
  f_b(2,0) = az;
  /* initialize the time */
  _tprev = time;
}

void ekfNavINS::ekf_update( uint64_t time, unsigned long TOW, double vn,double ve,double vd,double lat,double lon,double alt,
                          float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz ) {
  if (!initialized_) {
    ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
    // initialized flag
    initialized_ = true;
  } else {
    // get the change in time
    _dt = ((float)(time - _tprev)) / 1e6;
    _tprev = time;
    // Update Gyro and Accelerometer biases
    updateBias(ax, ay, az, p, q, r);
    // Update lat, lng, alt, velocity INS values to matrix
    lla_ins(0,0) = lat_ins;
    lla_ins(1,0) = lon_ins;
    lla_ins(2,0) = alt_ins;
    V_ins(0,0) = vn_ins;
    V_ins(1,0) = ve_ins;
    V_ins(2,0) = vd_ins;
    // Attitude Update
    dq(0) = 1.0f;
    dq(1) = 0.5f*om_ib(0,0)*_dt;
    dq(2) = 0.5f*om_ib(1,0)*_dt;
    dq(3) = 0.5f*om_ib(2,0)*_dt;
    quat = qmult(quat,dq);
    quat.normalize();
    // Avoid quaternion flips sign
    if (quat(0) < 0) {
      quat = -1.0f*quat;
    }
    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
    // obtain euler angles from quaternion
    std::tie(phi, theta, psi) = toEulerAngles(quat);
    // Velocity Update
    dx = C_B2N*f_b + grav;
    vn_ins += _dt*dx(0,0);
    ve_ins += _dt*dx(1,0);
    vd_ins += _dt*dx(2,0);
    // Position Update
    dxd = llarate(V_ins,lla_ins);
    lat_ins += _dt*dxd(0,0);
    lon_ins += _dt*dxd(1,0);
    alt_ins += _dt*dxd(2,0);
    // Jacobian update
    updateJacobianMatrix();
    // Update process noise and covariance time
    updateProcessNoiseCovarianceTime(_dt);
    // Gps measurement update
    if ((TOW - previousTOW) > 0) {
      previousTOW = TOW;
      lla_gps(0,0) = lat;
      lla_gps(1,0) = lon;
      lla_gps(2,0) = alt;
      V_gps(0,0) = vn;
      V_gps(1,0) = ve;
      V_gps(2,0) = vd;
      lla_ins(0,0) = lat_ins;
      lla_ins(1,0) = lon_ins;
      lla_ins(2,0) = alt_ins;
      V_ins(0,0) = vn_ins;
      V_ins(1,0) = ve_ins;
      V_ins(2,0) = vd_ins;
      // Create measurement Y
      updateCalculatedVsPredicted();
      // Kalman gain
      K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
      // Covariance update
      P = (Eigen::Matrix<float,15,15>::Identity()-K*H)*P*(Eigen::Matrix<float,15,15>::Identity()-K*H).transpose() + K*R*K.transpose();
      // State update
      x = K*y;
      // Update the results
      update15statesAfterKF();
    }
    // Get the new Specific forces and Rotation Rate
    updateBias(ax, ay, az, p, q, r);
  }
}

std::tuple<float,float,float> ekfNavINS::getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz) {
  // initial attitude and heading
  theta = asinf(ax/G);
  phi = -asinf(ay/(G*cosf(theta)));
  // magnetic heading correction due to roll and pitch angle
  Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
  Byc = hy*cosf(phi) - hz*sinf(phi);
  // finding initial heading
  psi = -atan2f(Byc,Bxc);
  return (std::make_tuple(theta,phi,psi));
}

void ekfNavINS::updateCalculatedVsPredicted() {
      // Position, converted to NED
      pos_ecef_ins = lla2ecef(lla_ins);
      pos_ecef_gps = lla2ecef(lla_gps);
      pos_ned_gps = ecef2ned(pos_ecef_gps - pos_ecef_ins, lla_ins);
      // Update the difference between calculated and predicted
      y(0,0) = (float)(pos_ned_gps(0,0));
      y(1,0) = (float)(pos_ned_gps(1,0));
      y(2,0) = (float)(pos_ned_gps(2,0));
      y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
      y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
      y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
}

void ekfNavINS::update15statesAfterKF() {
      estmimated_ins = llarate ((x.block(0,0,3,1)).cast<double>(), lat_ins, alt_ins);
      lat_ins += estmimated_ins(0,0);
      lon_ins += estmimated_ins(1,0);
      alt_ins += estmimated_ins(2,0);
      vn_ins = vn_ins + x(3,0);
      ve_ins = ve_ins + x(4,0);
      vd_ins = vd_ins + x(5,0);
      // Attitude correction
      dq(0,0) = 1.0f;
      dq(1,0) = x(6,0);
      dq(2,0) = x(7,0);
      dq(3,0) = x(8,0);
      quat = qmult(quat,dq);
      quat.normalize();
      // obtain euler angles from quaternion
      std::tie(phi, theta, psi) = toEulerAngles(quat);
      abx = abx + x(9,0);
      aby = aby + x(10,0);
      abz = abz + x(11,0);
      gbx = gbx + x(12,0);
      gby = gby + x(13,0);
      gbz = gbz + x(14,0);
}

void ekfNavINS::updateBias(float ax,float ay,float az,float p,float q, float r) {
  f_b(0,0) = ax - abx;
  f_b(1,0) = ay - aby;
  f_b(2,0) = az - abz;
  om_ib(0,0) = p - gbx;
  om_ib(1,0) = q - gby;
  om_ib(2,0) = r - gbz;
}

void ekfNavINS::updateProcessNoiseCovarianceTime(float _dt) {
  PHI = Eigen::Matrix<float,15,15>::Identity()+Fs*_dt;
  // Process Noise
  Gs.setZero();
  Gs.block(3,0,3,3) = -C_B2N;
  Gs.block(6,3,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  Gs.block(9,6,6,6) = Eigen::Matrix<float,6,6>::Identity();
  // Discrete Process Noise
  Q = PHI*_dt*Gs*Rw*Gs.transpose();
  Q = 0.5f*(Q+Q.transpose());
  // Covariance Time Update
  P = PHI*P*PHI.transpose()+Q;
  P = 0.5f*(P+P.transpose());
}

void ekfNavINS::updateJacobianMatrix() {
    // Jacobian
  Fs.setZero();
  // ... pos2gs
  Fs.block(0,3,3,3) = Eigen::Matrix<float,3,3>::Identity();
  // ... gs2pos
  Fs(5,2) = -2.0f*G/EARTH_RADIUS;
  // ... gs2att
  Fs.block(3,6,3,3) = -2.0f*C_B2N*sk(f_b);
  // ... gs2acc
  Fs.block(3,9,3,3) = -C_B2N;
  // ... att2att
  Fs.block(6,6,3,3) = -sk(om_ib);
  // ... att2gyr
  Fs.block(6,12,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  // ... Accel Markov Bias
  Fs.block(9,9,3,3) = -1.0f/TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Fs.block(12,12,3,3) = -1.0f/TAU_G*Eigen::Matrix<float,3,3>::Identity();
}

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix<float,3,3> ekfNavINS::sk(Eigen::Matrix<float,3,1> w) {
  Eigen::Matrix<float,3,3> C;
  C(0,0) = 0.0f;    C(0,1) = -w(2,0); C(0,2) = w(1,0);
  C(1,0) = w(2,0);  C(1,1) = 0.0f;    C(1,2) = -w(0,0);
  C(2,0) = -w(1,0); C(2,1) = w(0,0);  C(2,2) = 0.0f;
  return C;
}

constexpr std::pair<double, double> ekfNavINS::earthradius(double lat) {
  double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
  double Rew = EARTH_RADIUS / sqrt(denom);
  double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
  return (std::make_pair(Rew, Rns));
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
  double Rew, Rns, denom;
  Eigen::Matrix<double,3,1> lla_dot;
  std::tie(Rew, Rns) = earthradius(lla(0,0));
  lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
  lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
  lla_dot(2,0) = -V(2,0);
  return lla_dot;
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V, double lat, double alt) {
  Eigen::Matrix<double,3,1> lla;
  lla(0,0) = lat;
  lla(1,0) = 0.0; // Not used
  lla(2,0) = alt;
  return llarate(V, lla);
}

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
Eigen::Matrix<double,3,1> ekfNavINS::lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew, denom;
  Eigen::Matrix<double,3,1> ecef;
  std::tie(Rew, std::ignore) = earthradius(lla(0,0));
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
Eigen::Matrix<double,3,1> ekfNavINS::ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  return ned;
}

// quaternion to dcm
Eigen::Matrix<float,3,3> ekfNavINS::quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}

// quaternion multiplication
Eigen::Matrix<float,4,1> ekfNavINS::qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,4,1> r;
  r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
  r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
  r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
  r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
  return r;
}

// bound angle between -180 and 180
float ekfNavINS::constrainAngle180(float dta) {
  if(dta >  M_PI) dta -= (M_PI*2.0f);
  if(dta < -M_PI) dta += (M_PI*2.0f);
  return dta;
}

// bound angle between 0 and 360
float ekfNavINS::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

Eigen::Matrix<float,4,1> ekfNavINS::toQuaternion(float yaw, float pitch, float roll) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    Eigen::Matrix<float,4,1> q;
    q(0) = cr * cp * cy + sr * sp * sy; // w
    q(1) = cr * cp * sy - sr * sp * cy; // x
    q(2) = cr * sp * cy + sr * cp * sy; // y
    q(3) = sr * cp * cy - cr * sp * sy; // z
    return q;
}

std::tuple<float, float, float> ekfNavINS::toEulerAngles(Eigen::Matrix<float,4,1> quat) {
    float roll, pitch, yaw;
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0));
    float cosr_cosp = 1.0f - 2.0f * (quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0));
    roll = atan2f(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2.0f * (quat(0,0)*quat(2,0) - quat(1,0)*quat(3,0));
    //angles.pitch = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = asinf(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0));
    float cosy_cosp = 1.0f - 2.0f * (quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0));
    yaw = atan2f(siny_cosp, cosy_cosp);
    return std::make_tuple(roll, pitch, yaw);
}

