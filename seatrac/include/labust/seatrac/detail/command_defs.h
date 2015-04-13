struct StatusCmd: public SeatracMessage
{
typedef boost::shared_ptr< StatusCmd >  Ptr;
typedef boost::shared_ptr< StatusCmd const >  ConstPtr;
  enum {CID = 0x10};

  int getCid() const
{
  return StatusCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  StatusBits status_output;
};

struct PingSendCmd: public SeatracMessage
{
typedef boost::shared_ptr< PingSendCmd >  Ptr;
typedef boost::shared_ptr< PingSendCmd const >  ConstPtr;
  enum {CID = 0x40};

  int getCid() const
{
  return PingSendCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t dest;
  uint8_t msg_type;
};

struct DatSendCmd: public SeatracMessage
{
typedef boost::shared_ptr< DatSendCmd >  Ptr;
typedef boost::shared_ptr< DatSendCmd const >  ConstPtr;
  enum {CID = 0x60};

  int getCid() const
{
  return DatSendCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t dest;
  uint8_t msg_type;
  PayloadType data;
};

struct DatQueueSetCmd: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueSetCmd >  Ptr;
typedef boost::shared_ptr< DatQueueSetCmd const >  ConstPtr;
  enum {CID = 0x64};

  int getCid() const
{
  return DatQueueSetCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
  uint8_t dest;
  PayloadType data;
};

struct DatQueueClearCmd: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueClearCmd >  Ptr;
typedef boost::shared_ptr< DatQueueClearCmd const >  ConstPtr;
  enum {CID = 0x65};

  int getCid() const
{
  return DatQueueClearCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
};

struct DatQueueStatusCmd: public SeatracMessage
{
typedef boost::shared_ptr< DatQueueStatusCmd >  Ptr;
typedef boost::shared_ptr< DatQueueStatusCmd const >  ConstPtr;
  enum {CID = 0x66};

  int getCid() const
{
  return DatQueueStatusCmd::CID;
};
  bool isCommand() const;
  bool pack(SeatracMessage::DataBuffer& out) const;
  bool unpack(const SeatracMessage::DataBuffer& in);
};

