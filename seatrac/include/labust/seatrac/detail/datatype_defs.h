struct StatusBits
{
  bool ENVIRONMENT: 1;
  bool ATTITUDE: 1;
  bool MAG_CAL: 1;
  bool ACC_CAL: 1;
  bool AHRS_RAW_DATA: 1;
  bool AHRS_COMP_DATA: 1;
};
BOOST_STATIC_ASSERT((sizeof(StatusBits) == 1) && ("StatusBits structure is assumed as size 1 bytes."));

struct AcoFixBits
{
  bool RANGE_VALID: 1;
  bool USBL_VALID: 1;
  bool POSITION_VALID: 1;
  bool POSITION_ENHANCED: 1;
  bool POSITION_FLT_ERROR: 1;
};
BOOST_STATIC_ASSERT((sizeof(AcoFixBits) == 1) && ("AcoFixBits structure is assumed as size 1 bytes."));

struct MagCalibration
{
  uint8_t buffer;
  bool valid;
  uint32_t age;
  uint8_t fit;
};

struct EnvStatus
{
  uint16_t supply;
  int16_t temp;
  int32_t pressure;
  int32_t depth;
  uint16_t vos;
};

struct AccCalibration
{
  vec3si lim_min;
  vec3si lim_max;
};

struct AHRSData
{
  vec3si acc;
  vec3si mag;
  vec3si gyro;
};

struct RangeData
{
  uint32_t count;
  int32_t time;
  uint16_t dist;
};

struct USBLData
{
  USBLRSSIVec rssi;
  int16_t azimuth;
  int16_t elevation;
  int16_t fit_error;
};

struct AcoFix
{
  enum {RANGE_SC = 10};
  enum {ANGLE_SC = 10};

  uint8_t dest;
  AcoFixBits flags;
  uint8_t msg_type;
  vec3si attitude;
  uint16_t depth_local;
  uint16_t vos;
  int16_t rssi;
  RangeData range;
  USBLData usbl;
  vec3si position;
};

struct Status
{
  enum {YAW = 0};
  enum {PITCH = 1};
  enum {ROLL = 2};
  enum {ATT_SC = 10};
  enum {TEMP_SC = 10};
  enum {DEPTH_SC = 10};
  enum {VOS_SC = 10};
  enum {PRESSURE_SC = 1000};
  enum {VOLTAGE_SC = 1000};

  StatusBits status_output;
  uint64_t timestamp;
  EnvStatus env;
  vec3si attitude;
  MagCalibration mag_cal;
  AccCalibration acc;
  AHRSData ahrs_raw;
  AHRSData ahrs_comp;
};

struct AcoMsg
{
  uint8_t dest;
  uint8_t src;
  uint8_t msg_type;
  uint16_t depth;
  uint8_t payload_id;
  PayloadType payload;
};

