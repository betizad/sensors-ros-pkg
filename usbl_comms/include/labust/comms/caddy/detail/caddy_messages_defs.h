struct BuddyReport {
public:
  enum { MSG_ID = 1};

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t msg_id;
  int32_t offset_x;
  int32_t offset_y;
  int32_t course;
  uint8_t speed;
  uint8_t depth;
  uint8_t diver_offset_x;
  uint8_t diver_offset_y;

};


struct SurfaceNav {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t msg_id;
  int32_t course;
  int32_t speed;
  int32_t offset_x;
  int32_t offset_y;

};


struct DiverNav {
public:

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t msg_id;
  int32_t heading;
  int32_t depth;

};


