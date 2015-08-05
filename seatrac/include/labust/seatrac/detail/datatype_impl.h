  void StatusBits::pack(boost::archive::binary_oarchive& out) const
{
    uint8_t storage = 0;
  storage |= (uint8_t(ENVIRONMENT) & ((1<<1)-1)) << 0;
  storage |= (uint8_t(ATTITUDE) & ((1<<1)-1)) << 1;
  storage |= (uint8_t(MAG_CAL) & ((1<<1)-1)) << 2;
  storage |= (uint8_t(ACC_CAL) & ((1<<1)-1)) << 3;
  storage |= (uint8_t(AHRS_RAW_DATA) & ((1<<1)-1)) << 4;
  storage |= (uint8_t(AHRS_COMP_DATA) & ((1<<1)-1)) << 5;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<1; ++i,++pt) out << *pt;

};

  void StatusBits::unpack(boost::archive::binary_iarchive& in) 
{
    uint8_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<1; ++i,++pt) in >> *pt;
  ENVIRONMENT = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  ATTITUDE = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  MAG_CAL = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  ACC_CAL = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  AHRS_RAW_DATA = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  AHRS_COMP_DATA = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;

};




  void AcoFixBits::pack(boost::archive::binary_oarchive& out) const
{
    uint8_t storage = 0;
  storage |= (uint8_t(RANGE_VALID) & ((1<<1)-1)) << 0;
  storage |= (uint8_t(USBL_VALID) & ((1<<1)-1)) << 1;
  storage |= (uint8_t(POSITION_VALID) & ((1<<1)-1)) << 2;
  storage |= (uint8_t(POSITION_ENHANCED) & ((1<<1)-1)) << 3;
  storage |= (uint8_t(POSITION_FLT_ERROR) & ((1<<1)-1)) << 4;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<1; ++i,++pt) out << *pt;

};

  void AcoFixBits::unpack(boost::archive::binary_iarchive& in) 
{
    uint8_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<1; ++i,++pt) in >> *pt;
  RANGE_VALID = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  USBL_VALID = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  POSITION_VALID = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  POSITION_ENHANCED = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;
  POSITION_FLT_ERROR = static_cast<bool>(storage & ((1<<1)-1));
  storage >>= 1;

};




  void BitMessageTest::pack(boost::archive::binary_oarchive& out) const
{
    uint64_t storage = 0;
  storage |= (uint64_t(type) & ((1<<4)-1)) << 0;
  storage |= (uint64_t(lat) & ((1<<22)-1)) << 4;
  storage |= (uint64_t(lon) & ((1<<22)-1)) << 26;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<6; ++i,++pt) out << *pt;

};

  void BitMessageTest::unpack(boost::archive::binary_iarchive& in) 
{
    uint64_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<6; ++i,++pt) in >> *pt;
  type = static_cast<uint8_t>(storage & ((1<<4)-1));
  storage >>= 4;
  lat = static_cast<int32_t>(storage & ((1<<22)-1));
  storage >>= 22;
  lon = static_cast<int32_t>(storage & ((1<<22)-1));
  storage >>= 22;

};




  void MagCalibration::pack(boost::archive::binary_oarchive& out) const
{
    out <<buffer;
  out <<valid;
  out <<age;
  out <<fit;

};

  void MagCalibration::unpack(boost::archive::binary_iarchive& in) 
{
    in >> buffer;
  in >> valid;
  in >> age;
  in >> fit;

};




  void EnvStatus::pack(boost::archive::binary_oarchive& out) const
{
    out <<supply;
  out <<temp;
  out <<pressure;
  out <<depth;
  out <<vos;

};

  void EnvStatus::unpack(boost::archive::binary_iarchive& in) 
{
    in >> supply;
  in >> temp;
  in >> pressure;
  in >> depth;
  in >> vos;

};




  void AccCalibration::pack(boost::archive::binary_oarchive& out) const
{
        for(int i=0; i<3; ++i) out << lim_min[i];
      for(int i=0; i<3; ++i) out << lim_max[i];

};

  void AccCalibration::unpack(boost::archive::binary_iarchive& in) 
{
        for(int i=0; i<3; ++i) in >> lim_min[i];
      for(int i=0; i<3; ++i) in >> lim_max[i];

};




  void AHRSData::pack(boost::archive::binary_oarchive& out) const
{
        for(int i=0; i<3; ++i) out << acc[i];
      for(int i=0; i<3; ++i) out << mag[i];
      for(int i=0; i<3; ++i) out << gyro[i];

};

  void AHRSData::unpack(boost::archive::binary_iarchive& in) 
{
        for(int i=0; i<3; ++i) in >> acc[i];
      for(int i=0; i<3; ++i) in >> mag[i];
      for(int i=0; i<3; ++i) in >> gyro[i];

};




  void RangeData::pack(boost::archive::binary_oarchive& out) const
{
    out <<count;
  out <<time;
  out <<dist;

};

  void RangeData::unpack(boost::archive::binary_iarchive& in) 
{
    in >> count;
  in >> time;
  in >> dist;

};




  void USBLData::pack(boost::archive::binary_oarchive& out) const
{
      uint8_t rssi_len(rssi.size());
  out << rssi_len;
  for(int i=0; i<rssi.size(); ++i) out << rssi[i];
  out <<azimuth;
  out <<elevation;
  out <<fit_error;

};

  void USBLData::unpack(boost::archive::binary_iarchive& in) 
{
      uint8_t rssi_len;
in >> rssi_len;
  rssi.resize(rssi_len);
  for(int i=0; i<rssi.size(); ++i) in >> rssi[i];
  in >> azimuth;
  in >> elevation;
  in >> fit_error;

};




  void AcoFix::pack(boost::archive::binary_oarchive& out) const
{
    out <<dest;
  out <<src;
  flags.pack(out);
  out <<msg_type;
      for(int i=0; i<3; ++i) out << attitude[i];
  out <<depth_local;
  out <<vos;
  out <<rssi;
  if (flags.RANGE_VALID){
  range.pack(out);
  }
  if (flags.USBL_VALID){
  usbl.pack(out);
  }
  if (flags.POSITION_VALID){
      for(int i=0; i<3; ++i) out << position[i];
  }

};

  void AcoFix::unpack(boost::archive::binary_iarchive& in) 
{
    in >> dest;
  in >> src;
  flags.unpack(in);
  in >> msg_type;
      for(int i=0; i<3; ++i) in >> attitude[i];
  in >> depth_local;
  in >> vos;
  in >> rssi;
  if (flags.RANGE_VALID){
  range.unpack(in);
  }
  if (flags.USBL_VALID){
  usbl.unpack(in);
  }
  if (flags.POSITION_VALID){
      for(int i=0; i<3; ++i) in >> position[i];
  }

};




  void Status::pack(boost::archive::binary_oarchive& out) const
{
    status_output.pack(out);
  out <<timestamp;
  if (status_output.ENVIRONMENT){
  env.pack(out);
  }
  if (status_output.ATTITUDE){
      for(int i=0; i<3; ++i) out << attitude[i];
  }
  if (status_output.MAG_CAL){
  mag_cal.pack(out);
  }
  if (status_output.ACC_CAL){
  acc.pack(out);
  }
  if (status_output.AHRS_RAW_DATA){
  ahrs_raw.pack(out);
  }
  if (status_output.AHRS_COMP_DATA){
  ahrs_comp.pack(out);
  }

};

  void Status::unpack(boost::archive::binary_iarchive& in) 
{
    status_output.unpack(in);
  in >> timestamp;
  if (status_output.ENVIRONMENT){
  env.unpack(in);
  }
  if (status_output.ATTITUDE){
      for(int i=0; i<3; ++i) in >> attitude[i];
  }
  if (status_output.MAG_CAL){
  mag_cal.unpack(in);
  }
  if (status_output.ACC_CAL){
  acc.unpack(in);
  }
  if (status_output.AHRS_RAW_DATA){
  ahrs_raw.unpack(in);
  }
  if (status_output.AHRS_COMP_DATA){
  ahrs_comp.unpack(in);
  }

};




  void AcoMsg::pack(boost::archive::binary_oarchive& out) const
{
    out <<dest;
  out <<src;
  out <<msg_type;
  out <<depth;
  out <<payload_id;
    uint8_t payload_len(payload.size());
  out << payload_len;
  for(int i=0; i<payload.size(); ++i) out << payload[i];

};

  void AcoMsg::unpack(boost::archive::binary_iarchive& in) 
{
    in >> dest;
  in >> src;
  in >> msg_type;
  in >> depth;
  in >> payload_id;
    uint8_t payload_len;
in >> payload_len;
  payload.resize(payload_len);
  for(int i=0; i<payload.size(); ++i) in >> payload[i];

};




