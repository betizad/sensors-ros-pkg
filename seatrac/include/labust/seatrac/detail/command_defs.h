struct StatusCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< StatusCmd >  Ptr;
  typedef boost::shared_ptr< StatusCmd const >  ConstPtr;
  enum { CID = 0x10};

  int getCid() const
{
  return StatusCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  StatusBits status_output;

};


struct PingSendCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< PingSendCmd >  Ptr;
  typedef boost::shared_ptr< PingSendCmd const >  ConstPtr;
  enum { CID = 0x40};

  int getCid() const
{
  return PingSendCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t dest;
  uint8_t msg_type;

};


struct DatSendCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< DatSendCmd >  Ptr;
  typedef boost::shared_ptr< DatSendCmd const >  ConstPtr;
  enum { CID = 0x60};

  int getCid() const
{
  return DatSendCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t dest;
  uint8_t msg_type;
  std::vector< uint8_t > data;

};


struct DatQueueSetCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< DatQueueSetCmd >  Ptr;
  typedef boost::shared_ptr< DatQueueSetCmd const >  ConstPtr;
  enum { CID = 0x64};

  int getCid() const
{
  return DatQueueSetCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;

  uint8_t dest;
  std::vector< uint8_t > data;

};


struct DatQueueClearCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< DatQueueClearCmd >  Ptr;
  typedef boost::shared_ptr< DatQueueClearCmd const >  ConstPtr;
  enum { CID = 0x65};

  int getCid() const
{
  return DatQueueClearCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;


};


struct DatQueueStatusCmd : public SeatracMessage {
public:
  typedef boost::shared_ptr< DatQueueStatusCmd >  Ptr;
  typedef boost::shared_ptr< DatQueueStatusCmd const >  ConstPtr;
  enum { CID = 0x66};

  int getCid() const
{
  return DatQueueStatusCmd::CID;
};

  bool isCommand() const;

  void pack(boost::archive::binary_oarchive& out) const;

  void unpack(boost::archive::binary_iarchive& in) ;


};


