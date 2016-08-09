struct BuddyReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t inited;
  double origin_lat;
  double origin_lon;
  uint8_t has_position;
  double north;
  double east;
  double depth;
  double altitude;
  double course;
  double speed;
  uint8_t has_diver;
  double diver_north;
  double diver_east;
  uint8_t battery_status;
  uint8_t leak_info;
  uint8_t command;
  double north_origin;
  double east_origin;
  double lawn_width;
  double lawn_length;

};


struct SurfaceReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t is_master;
  uint8_t inited;
  double origin_lat;
  double origin_lon;
  double north;
  double east;
  uint8_t has_diver;
  double diver_north;
  double diver_east;
  uint8_t command;
  double course;
  double speed;
  double north_origin;
  double east_origin;
  double lawn_width;
  double lawn_length;
  uint8_t predefined_chat;
  std::vector< uint8_t > chat;

};


struct DiverReport {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  double heading;
  double depth;
  double avg_flipper_rate;
  double hearth_rate;
  uint8_t optional_data;
  double breathing_rate;
  double motion_rate;
  double pad_space;
  uint8_t alarms;
  uint8_t predefined_chat;
  uint8_t command;
  double north_origin;
  double east_origin;
  double lawn_width;
  double lawn_length;
  std::vector< uint8_t > chat;

};


