

  bool StatusCmd::isCommand() const
{
  return true;
};

  void StatusCmd::pack(boost::archive::binary_oarchive& out) const
{
    status_output.pack(out);

};

  void StatusCmd::unpack(boost::archive::binary_iarchive& in) 
{
    status_output.unpack(in);

};






  bool PingSendCmd::isCommand() const
{
  return true;
};

  void PingSendCmd::pack(boost::archive::binary_oarchive& out) const
{
    out <<dest;
  out <<msg_type;

};

  void PingSendCmd::unpack(boost::archive::binary_iarchive& in) 
{
    in >> dest;
  in >> msg_type;

};






  bool DatSendCmd::isCommand() const
{
  return true;
};

  void DatSendCmd::pack(boost::archive::binary_oarchive& out) const
{
    out <<dest;
  out <<msg_type;
    uint8_t data_len(data.size());
  out << data_len;
  for(int i=0; i<data.size(); ++i) out << data[i];

};

  void DatSendCmd::unpack(boost::archive::binary_iarchive& in) 
{
    in >> dest;
  in >> msg_type;
    uint8_t data_len;
in >> data_len;
  data.resize(data_len);
  for(int i=0; i<data.size(); ++i) in >> data[i];

};






  bool DatQueueSetCmd::isCommand() const
{
  return true;
};

  void DatQueueSetCmd::pack(boost::archive::binary_oarchive& out) const
{
    out <<dest;
    uint8_t data_len(data.size());
  out << data_len;
  for(int i=0; i<data.size(); ++i) out << data[i];

};

  void DatQueueSetCmd::unpack(boost::archive::binary_iarchive& in) 
{
    in >> dest;
    uint8_t data_len;
in >> data_len;
  data.resize(data_len);
  for(int i=0; i<data.size(); ++i) in >> data[i];

};






  bool DatQueueClearCmd::isCommand() const
{
  return true;
};

  void DatQueueClearCmd::pack(boost::archive::binary_oarchive& out) const
{
  
};

  void DatQueueClearCmd::unpack(boost::archive::binary_iarchive& in) 
{
  
};






  bool DatQueueStatusCmd::isCommand() const
{
  return true;
};

  void DatQueueStatusCmd::pack(boost::archive::binary_oarchive& out) const
{
  
};

  void DatQueueStatusCmd::unpack(boost::archive::binary_iarchive& in) 
{
  
};




