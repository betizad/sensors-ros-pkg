struct StatusBits {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  bool ENVIRONMENT;
  bool ATTITUDE;
  bool MAG_CAL;
  bool ACC_CAL;
  bool AHRS_RAW_DATA;
  bool AHRS_COMP_DATA;

};


struct AcoFixBits {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  bool RANGE_VALID;
  bool USBL_VALID;
  bool POSITION_VALID;
  bool POSITION_ENHANCED;
  bool POSITION_FLT_ERROR;

};


struct BitMessageTest {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t type;
  int32_t lat;
  int32_t lon;

};


struct MagCalibration {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t buffer;
  uint8_t valid;
  uint32_t age;
  uint8_t fit;

};


struct EnvStatus {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint16_t supply;
  int16_t temp;
  int32_t pressure;
  int32_t depth;
  uint16_t vos;

};


struct AccCalibration {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  int16_t lim_min[3];
  int16_t lim_max[3];

};


struct AHRSData {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  int16_t acc[3];
  int16_t mag[3];
  int16_t gyro[3];

};


struct RangeData {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint32_t count;
  int32_t time;
  uint16_t dist;

};


struct USBLData {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  std::vector< int16_t > rssi;
  int16_t azimuth;
  int16_t elevation;
  int16_t fit_error;

};


struct AcoFix {
public:
  enum { RANGE_SC = 10};
  enum { ANGLE_SC = 10};
  enum { x = 1};
  enum { y = 0};
  enum { z = 2};

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t dest;
  uint8_t src;
  AcoFixBits flags;
  uint8_t msg_type;
  int16_t attitude[3];
  uint16_t depth_local;
  uint16_t vos;
  int16_t rssi;
  RangeData range;
  USBLData usbl;
  int16_t position[3];

};


struct Status {
public:
  enum { YAW = 0};
  enum { PITCH = 1};
  enum { ROLL = 2};
  enum { ATT_SC = 10};
  enum { TEMP_SC = 10};
  enum { DEPTH_SC = 10};
  enum { VOS_SC = 10};
  enum { PRESSURE_SC = 1000};
  enum { VOLTAGE_SC = 1000};

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  StatusBits status_output;
  uint64_t timestamp;
  EnvStatus env;
  int16_t attitude[3];
  MagCalibration mag_cal;
  AccCalibration acc;
  AHRSData ahrs_raw;
  AHRSData ahrs_comp;

};


struct AcoMsg {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t dest;
  uint8_t src;
  uint8_t msg_type;
  uint16_t depth;
  uint8_t payload_id;
  std::vector< uint8_t > payload;

};


