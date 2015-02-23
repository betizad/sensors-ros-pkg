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

struct DatSendResp: public SeatracMessage
{
typedef boost::shared_ptr< DatSendResp >  Ptr;
typedef boost::shared_ptr< DatSendResp const >  ConstPtr;
  enum {CID = 0x60};

  int getCid() const
{
  return DatSendResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
  uint8_t beacon_id;
};

struct DatReceive: public SeatracMessage
{
typedef boost::shared_ptr< DatReceive >  Ptr;
typedef boost::shared_ptr< DatReceive const >  ConstPtr;
  enum {CID = 0x61};

  int getCid() const
{
  return DatReceive::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  AcoFix acofix;
  uint8_t ack_flag;
  PayloadType data;
};

struct DatError: public SeatracMessage
{
typedef boost::shared_ptr< DatError >  Ptr;
typedef boost::shared_ptr< DatError const >  ConstPtr;
  enum {CID = 0x63};

  int getCid() const
{
  return DatError::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
  uint8_t beacon_id;
};

struct DatQueueSetResp: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueSetResp >  Ptr;
typedef boost::shared_ptr< DatQueueSetResp const >  ConstPtr;
  enum {CID = 0x64};

  int getCid() const
{
  return DatQueueSetResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
};

struct DatQueueClearResp: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueClearResp >  Ptr;
typedef boost::shared_ptr< DatQueueClearResp const >  ConstPtr;
  enum {CID = 0x65};

  int getCid() const
{
  return DatQueueClearResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t status;
};

struct DatQueueStatusResp: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueStatusResp >  Ptr;
typedef boost::shared_ptr< DatQueueStatusResp const >  ConstPtr;
  enum {CID = 0x66};

  int getCid() const
{
  return DatQueueStatusResp::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t packet_len;
};

