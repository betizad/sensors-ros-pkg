struct PingSendCmd: public SeatracMessage
{
  enum {CID = 0x40};

  int getCid() const
{
  return PingSendCmd::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t dest;
  uint8_t msg_type;
};

struct StatusCmd: public SeatracMessage
{
  enum {CID = 0x10};

  int getCid() const
{
  return StatusCmd::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  StatusBits status_output;
};

