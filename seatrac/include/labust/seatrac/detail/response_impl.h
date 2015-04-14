

  bool StatusResp::isCommand() const
{
  return false;
};

  void StatusResp::pack(boost::archive::binary_oarchive& out) const
{
    status.pack(out);

};

  void StatusResp::unpack(boost::archive::binary_iarchive& in) 
{
    status.unpack(in);

};






  bool PingSendResp::isCommand() const
{
  return false;
};

  void PingSendResp::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;
  out <<beacon_id;

};

  void PingSendResp::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;
  in >> beacon_id;

};






  bool PingReq::isCommand() const
{
  return false;
};

  void PingReq::pack(boost::archive::binary_oarchive& out) const
{
    acofix.pack(out);

};

  void PingReq::unpack(boost::archive::binary_iarchive& in) 
{
    acofix.unpack(in);

};






  bool PingResp::isCommand() const
{
  return false;
};

  void PingResp::pack(boost::archive::binary_oarchive& out) const
{
    acofix.pack(out);

};

  void PingResp::unpack(boost::archive::binary_iarchive& in) 
{
    acofix.unpack(in);

};






  bool PingError::isCommand() const
{
  return false;
};

  void PingError::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;
  out <<beacon_id;

};

  void PingError::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;
  in >> beacon_id;

};






  bool DatSendResp::isCommand() const
{
  return false;
};

  void DatSendResp::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;
  out <<beacon_id;

};

  void DatSendResp::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;
  in >> beacon_id;

};






  bool DatReceive::isCommand() const
{
  return false;
};

  void DatReceive::pack(boost::archive::binary_oarchive& out) const
{
    acofix.pack(out);
  out <<ack_flag;
    uint8_t data_len(data.size());
  out << data_len;
  for(int i=0; i<data.size(); ++i) out << data[i];

};

  void DatReceive::unpack(boost::archive::binary_iarchive& in) 
{
    acofix.unpack(in);
  in >> ack_flag;
    uint8_t data_len;
in >> data_len;
  data.resize(data_len);
  for(int i=0; i<data.size(); ++i) in >> data[i];

};






  bool DatError::isCommand() const
{
  return false;
};

  void DatError::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;
  out <<beacon_id;

};

  void DatError::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;
  in >> beacon_id;

};






  bool DatQueueSetResp::isCommand() const
{
  return false;
};

  void DatQueueSetResp::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;

};

  void DatQueueSetResp::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;

};






  bool DatQueueClearResp::isCommand() const
{
  return false;
};

  void DatQueueClearResp::pack(boost::archive::binary_oarchive& out) const
{
    out <<status;

};

  void DatQueueClearResp::unpack(boost::archive::binary_iarchive& in) 
{
    in >> status;

};






  bool DatQueueStatusResp::isCommand() const
{
  return false;
};

  void DatQueueStatusResp::pack(boost::archive::binary_oarchive& out) const
{
    out <<packet_len;

};

  void DatQueueStatusResp::unpack(boost::archive::binary_iarchive& in) 
{
    in >> packet_len;

};




