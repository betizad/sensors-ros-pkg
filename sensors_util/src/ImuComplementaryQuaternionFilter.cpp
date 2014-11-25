#include <labust/sensors/ImuComplementaryQuaternionFilter.hpp>
#include <labust/tools/conversions.hpp>

using namespace labust::sensors;

ImuComplementaryQuaternionFilter::ImuComplementaryQuaternionFilter(const int num_nodes, const double dT, const double beta, const double gyro_gain) :
    num_nodes(num_nodes),
    dT(dT),
    fs(1.f/dT),
    beta(beta),
    gyro_gain(gyro_gain),
    is_initialized(false),
    q(num_nodes) {
  // Initialize quaternion for all nodes to {1,0,0,0}
  // TODO(irendulic): consider using Eigen quaternion.
  // TODO(irendulic): initialization based on first frame 
  // accelerometer and magnetometer measurement.
  for (int i=0; i<num_nodes; ++i) {
    q[i].resize(4);
    q[i][0] = 1.0;
    q[i][1] = 0;
    q[i][2] = 0;
    q[i][3] = 0;
  }
}

ImuComplementaryQuaternionFilter::~ImuComplementaryQuaternionFilter() {}

void ImuComplementaryQuaternionFilter::processFrame(const Eigen::MatrixXd data) {
  if (!is_initialized) {
    this->initialOrientation(data);
    is_initialized = true;
    return;
  }
  for (int i=0; i<num_nodes; ++i) {
    double ax = data(i,0);
    double ay = data(i,1);
    double az = data(i,2);
    // Gyro data must be normalized to rad/s. Accelerometer and magnetometer
    // data can be in any range as long as all axes have the same scale.
    double gx = data(i,6) * gyro_gain;
    double gy = data(i,7) * gyro_gain;
    double gz = data(i,8) * gyro_gain;
    double mx = data(i,3);
    double my = data(i,4);
    double mz = data(i,5);
    double q0 = q[i][0];
    double q1 = q[i][1];
    double q2 = q[i][2];
    double q3 = q[i][3];

    // Rate of change of quaternion from gyroscope data
    double qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    double qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    double qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    double qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

    double recipNorm;
    // Normalize accelerometer measurement
    recipNorm = 1.f / sqrt(ax*ax + ay*ay + az*az);
    ax = ax * recipNorm;
    ay = ay * recipNorm;
    az = az * recipNorm;
    // Normalize magnetometer measurement
    recipNorm = 1.f / sqrt(mx*mx + my*my + mz*mz);
    mx = mx * recipNorm;
    my = my * recipNorm;
    mz = mz * recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    double u2q0mx = 2.0 * q0 * mx;
    double u2q0my = 2.0 * q0 * my;
    double u2q0mz = 2.0 * q0 * mz;
    double u2q1mx = 2.0 * q1 * mx;
    double u2q0 = 2.0 * q0;
    double u2q1 = 2.0 * q1;
    double u2q2 = 2.0 * q2;
    double u2q3 = 2.0 * q3;
    double u2q0q2 = 2.0 * q0 * q2;
    double u2q2q3 = 2.0 * q2 * q3;
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    double hx = mx*q0q0 - u2q0my*q3 + u2q0mz*q2 + mx*q1q1 + u2q1*my*q2 + u2q1*mz*q3 - mx*q2q2 - mx*q3q3;
    double hy = u2q0mx*q3 + my*q0q0 - u2q0mz*q1 + u2q1mx*q2 - my*q1q1 + my*q2q2 + u2q2*mz*q3 - my*q3q3;
    double u2bx = sqrt(hx*hx + hy*hy);
    double u2bz = -u2q0mx*q2 + u2q0my*q1 + mz*q0q0 + u2q1mx*q3 - mz*q1q1 + u2q2*my*q3 - mz*q2q2 + mz*q3q3;
    double u4bx = 2.0 * u2bx;
    double u4bz = 2.0 * u2bz;

    // Gradient decent algorithm corrective step
    double s0 = -u2q2 * (2.0*q1q3 - u2q0q2 - ax) + u2q1 * (2.0*q0q1 + u2q2q3 - ay) 
        - u2bz * q2 * (u2bx * (0.5 - q2q2 - q3q3) + u2bz * (q1q3 - q0q2) - mx) 
        + (-u2bx*q3 + u2bz*q1) * (u2bx * (q1q2 - q0q3) + u2bz * (q0q1 + q2q3) - my) 
        + u2bx*q2 * (u2bx * (q0q2 + q1q3) + u2bz * (0.5 - q1q1 - q2q2) - mz);
    double s1 = u2q3 * (2.0*q1q3 - u2q0q2 - ax) + u2q0 * (2.0*q0q1 + u2q2q3 - ay) 
        - 4.0 * q1 * (1 - 2.0*q1q1 - 2.0*q2q2 - az) + u2bz * q3 * (u2bx * (0.5 - q2q2 - q3q3) 
        + u2bz * (q1q3 - q0q2) - mx) + (u2bx*q2 + u2bz*q0) * (u2bx * (q1q2 - q0q3) + u2bz * (q0q1 + q2q3) - my) 
        + (u2bx*q3 - u4bz*q1) * (u2bx * (q0q2 + q1q3) + u2bz * (0.5 - q1q1 - q2q2) - mz);
    double s2 = -u2q0 * (2.0*q1q3 - u2q0q2 - ax) + u2q3 * (2.0*q0q1 + u2q2q3 - ay) 
        - 4.0 * q2 * (1 - 2.0*q1q1 - 2.0*q2q2 - az) + (-u4bx*q2 - u2bz*q0) * (u2bx * (0.5 - q2q2 - q3q3) 
        + u2bz * (q1q3 - q0q2) - mx) + (u2bx*q1 + u2bz*q3) * (u2bx * (q1q2 - q0q3) + u2bz * (q0q1 + q2q3) - my) 
        + (u2bx*q0 - u4bz * q2) * (u2bx * (q0q2 + q1q3) + u2bz * (0.5 - q1q1 - q2q2) - mz);
    double s3 = u2q1 * (2.0*q1q3 - u2q0q2 - ax) + u2q2 * (2.0*q0q1 + u2q2q3 - ay) 
        + (-u4bx*q3 + u2bz*q1) * (u2bx * (0.5 - q2q2 - q3q3) + u2bz * (q1q3 - q0q2) - mx) 
        + (-u2bx*q0 + u2bz*q2) * (u2bx * (q1q2 - q0q3) + u2bz * (q0q1 + q2q3) - my) 
        + u2bx * q1 * (u2bx * (q0q2 + q1q3) + u2bz * (0.5 - q1q1 - q2q2) - mz);

    // Normalize step magnitude
    recipNorm = 1.f / sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 = s0 * recipNorm;
    s1 = s1 * recipNorm;
    s2 = s2 * recipNorm;
    s3 = s3 * recipNorm;

    // Apply feedback step
    qDot1 = qDot1 - (beta * s0);
    qDot2 = qDot2 - (beta * s1);
    qDot3 = qDot3 - (beta * s2);
    qDot4 = qDot4 - (beta * s3);


    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dT;
    q1 += qDot2 * dT;
    q2 += qDot3 * dT;
    q3 += qDot4 * dT;

    // Normalize quaternion
    recipNorm = 1.f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * recipNorm;
    q1 = q1 * recipNorm;
    q2 = q2 * recipNorm;
    q3 = q3 * recipNorm;

    q[i][0] = q0;
    q[i][1] = q1;
    q[i][2] = q2;
    q[i][3] = q3;
  }
}

void ImuComplementaryQuaternionFilter::initialOrientation(const Eigen::MatrixXd data) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
    for (int i=0; i<num_nodes; ++i) {
      double roll = atan2(data(i,ay), data(i,az));
      double pitch = -atan2(data(i,ax), sqrt(data(i,ay) * data(i,ay) + data(i,az) * data(i,az)));
      double hx = data(i,mx) * cos(pitch) + data(i,my) * sin(pitch) * sin(roll) + data(i,mz) * cos(roll) * sin(pitch);
      double hy = data(i,my) * cos(roll) - data(i,mz) * sin(roll);
      double yaw = atan2(-hy, hx);
  
      Eigen::Quaternion<double> t;
      labust::tools::quaternionFromEulerZYX(roll, pitch, yaw, t);

      q[i][0] = t.w();
      q[i][1] = t.x();
      q[i][2] = t.y();
      q[i][3] = t.z();
    }
}

std::vector<std::vector<double> > ImuComplementaryQuaternionFilter::getQuaternionOrientation() {
  return q;
}

void ImuComplementaryQuaternionFilter::printQuaternionOrientation() {
  for (int i=0; i<num_nodes; ++i) {
    std::cout << "NODE " << i << " : ";
    for (int j=0; j<4; ++j) {
      std::cout << q[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
