struct PingSendCmd: public SeatracMessage
{
typedef boost::shared_ptr< PingSendCmd >  Ptr;
typedef boost::shared_ptr< PingSendCmd const >  ConstPtr;
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

struct DataSendCmd: public SeatracMessage
{
typedef boost::shared_ptr< DataSendCmd >  Ptr;
typedef boost::shared_ptr< DataSendCmd const >  ConstPtr;
  enum {CID = 0x60};

  int getCid() const
{
  return DataSendCmd::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t dest;
  uint8_t msg_type;
  PayloadType data;
};

struct StatusCmd: public SeatracMessage
{
typedef boost::shared_ptr< StatusCmd >  Ptr;
typedef boost::shared_ptr< StatusCmd const >  ConstPtr;
  enum {CID = 0x10};

  int getCid() const
{
  return StatusCmd::CID;
};
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  StatusBits status_output;
};

