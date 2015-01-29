BOOST_CLASS_IMPLEMENTATION(labust::seatrac::StatusBits, boost::serialization::primitive_type)

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AcoFixBits, boost::serialization::primitive_type)

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::MagCalibration, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::MagCalibration& object, const unsigned int version)
{
ar & object.buffer;
ar & object.valid;
ar & object.age;
ar & object.fit;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::EnvStatus, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::EnvStatus& object, const unsigned int version)
{
ar & object.supply;
ar & object.temp;
ar & object.pressure;
ar & object.depth;
ar & object.vos;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AccCalibration, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::AccCalibration& object, const unsigned int version)
{
ar & object.lim_min;
ar & object.lim_max;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AHRSData, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::AHRSData& object, const unsigned int version)
{
ar & object.acc;
ar & object.mag;
ar & object.gyro;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::RangeData, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::RangeData& object, const unsigned int version)
{
ar & object.count;
ar & object.time;
ar & object.dist;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::USBLData, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::USBLData& object, const unsigned int version)
{
ar & object.rssi;
ar & object.azimuth;
ar & object.elevation;
ar & object.fit_error;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AcoFix, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::AcoFix& object, const unsigned int version)
{
ar & object.dest;
ar & object.flags;
ar & object.msg_type;
ar & object.attitude;
ar & object.depth_local;
ar & object.vos;
ar & object.rssi;
if (object.flags.RANGE_VALID) ar & object.range;
if (object.flags.USBL_VALID) ar & object.usbl;
if (object.flags.POSITION_VALID) ar & object.position;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::Status, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::Status& object, const unsigned int version)
{
ar & object.status_output;
ar & object.timestamp;
if (object.status_output.ENVIRONMENT) ar & object.env;
if (object.status_output.ATTITUDE) ar & object.attitude;
if (object.status_output.MAG_CAL) ar & object.mag_cal;
if (object.status_output.ACC_CAL) ar & object.acc;
if (object.status_output.AHRS_RAW_DATA) ar & object.ahrs_raw;
if (object.status_output.AHRS_COMP_DATA) ar & object.ahrs_comp;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::AcoMsg, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::AcoMsg& object, const unsigned int version)
{
ar & object.dest;
ar & object.src;
ar & object.msg_type;
ar & object.depth;
ar & object.payload_id;
ar & object.payload;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::PingSendCmd, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::PingSendCmd& object, const unsigned int version)
{
ar & object.dest;
ar & object.msg_type;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::DataSendCmd, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::DataSendCmd& object, const unsigned int version)
{
ar & object.dest;
ar & object.msg_type;
ar & object.data;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::StatusCmd, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::StatusCmd& object, const unsigned int version)
{
ar & object.status_output;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::StatusResp, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::StatusResp& object, const unsigned int version)
{
ar & object.status;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::PingSendResp, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::PingSendResp& object, const unsigned int version)
{
ar & object.status;
ar & object.beacon_id;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::DataSendResp, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::DataSendResp& object, const unsigned int version)
{
ar & object.status;
ar & object.beacon_id;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::PingReq, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::PingReq& object, const unsigned int version)
{
ar & object.acofix;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::PingResp, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::PingResp& object, const unsigned int version)
{
ar & object.acofix;
}}};

BOOST_CLASS_IMPLEMENTATION(labust::seatrac::PingError, boost::serialization::object_serializable)
namespace boost { namespace serialization {
template<class Archive>
void serialize(Archive& ar, labust::seatrac::PingError& object, const unsigned int version)
{
ar & object.status;
ar & object.beacon_id;
}}};

