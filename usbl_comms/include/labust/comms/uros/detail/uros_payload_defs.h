struct RhodamineData {
public:
  enum { DEPTH_SC = 2};

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t status_flag;
  int32_t lat;
  int32_t lon;
  uint8_t depth;
  uint8_t adc_gain;
  uint16_t adc;

};


struct LupisUpdate {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t cmd_flag;
  int32_t lat;
  int32_t lon;

};


