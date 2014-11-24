struct StatusResp: public SeatracMessage
{
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

