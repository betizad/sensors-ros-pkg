#ifndef IMU_FILTER_MADWICK_IMU_FILTER_H
#define IMU_FILTER_MADWICK_IMU_FILTER_H

class ImuFilter {
 public:

  ImuFilter();
  void reset();
  virtual ~ImuFilter();
  
  void setAlgorithmGain(double gain) {
      gain_ = gain;
  }

  void setDriftBiasGain(double zeta) {
      zeta_ = zeta;
  }

  void getOrientation(double& q0, double& q1, double& q2, double& q3) {
    q0 = this->q0;
    q1 = this->q1;
    q2 = this->q2;
    q3 = this->q3;
  }

  void setOrientation(double q0, double q1, double q2, double q3) {
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;

    w_bx_ = 0;
    w_by_ = 0;
    w_bz_ = 0;
  }

  void madgwickAHRSupdate(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

  void madgwickAHRSupdateIMU(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float dt);
 
 private:
  void getInitialOrientation(float gx, float gy, float gz, 
                             float ax, float ay, float az, 
                             float mx, float my, float mz);
  double gain_;     // algorithm gain
  double zeta_;	  // gyro drift bias gain
  double q0, q1, q2, q3;  // quaternion
  float w_bx_, w_by_, w_bz_;  
  bool is_initialized_;
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_H
