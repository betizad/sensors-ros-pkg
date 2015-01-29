struct StatusResp: public SeatracMessage
{
typedef boost::shared_ptr< StatusResp >  Ptr;
typedef boost::shared_ptr< StatusResp const >  ConstPtr;
  enum {CID = 0x10};

  int getCid() const
{
  return StatusResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  Status status;
};

struct PingSendResp: public SeatracMessage
{
typedef boost::shared_ptr< PingSendResp >  Ptr;
typedef boost::shared_ptr< PingSendResp const >  ConstPtr;
  enum {CID = 0x40};

  int getCid() const
{
  return PingSendResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
  uint8_t beacon_id;
};

struct DataSendResp: public SeatracMessage
{
typedef boost::shared_ptr< DataSendResp >  Ptr;
typedef boost::shared_ptr< DataSendResp const >  ConstPtr;
  enum {CID = 0x60};

  int getCid() const
{
  return DataSendResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
  uint8_t beacon_id;
};

struct PingReq: public SeatracMessage
{
typedef boost::shared_ptr< PingReq >  Ptr;
typedef boost::shared_ptr< PingReq const >  ConstPtr;
  enum {CID = 0x41};

  int getCid() const
{
  return PingReq::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  AcoFix acofix;
};

struct PingResp: public SeatracMessage
{
typedef boost::shared_ptr< PingResp >  Ptr;
typedef boost::shared_ptr< PingResp const >  ConstPtr;
  enum {CID = 0x42};

  int getCid() const
{
  return PingResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  AcoFix acofix;
};

struct PingError: public SeatracMessage
{
typedef boost::shared_ptr< PingError >  Ptr;
typedef boost::shared_ptr< PingError const >  ConstPtr;
  enum {CID = 0x43};

  int getCid() const
{
  return PingError::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
  uint8_t beacon_id;
};

