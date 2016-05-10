struct BuddyReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double offset_x;
  double offset_y;
  double course;
  double speed;
  double depth;
  double altitude;
  uint8_t battery_info;
  uint8_t leak_info;
  uint8_t mission_status;
  double diver_offset_x;
  double diver_offset_y;

};


struct SurfaceReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double offset_x;
  double offset_y;
  double course;
  double speed;
  uint8_t mission_cmd;
  double lawn_width;
  double lawn_length;

};


struct SurfaceChat {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double offset_x;
  double offset_y;
  double course;
  double speed;
  uint8_t chat[5];

};


struct DiverReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double heading;
  double depth;
  double paddle_rate;
  double hearth_rate;
  uint8_t mission_cmd;

};


struct DiverChat {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double paddle_rate;
  double hearth_rate;
  uint8_t chat[8];

};


