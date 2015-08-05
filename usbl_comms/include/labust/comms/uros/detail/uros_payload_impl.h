  void RhodamineData::pack(boost::archive::binary_oarchive& out) const
{
    uint32_t storage = 0;
  storage |= (uint32_t(status_flag) & ((1<<1)-1)) << 0;
  storage |= (uint32_t(lat) & ((1<<10)-1)) << 1;
  storage |= (uint32_t(lon) & ((1<<10)-1)) << 11;
  storage |= (uint32_t(depth) & ((1<<4)-1)) << 21;
  storage |= (uint32_t(adc_gain) & ((1<<0)-1)) << 25;
  storage |= (uint32_t(adc) & ((1<<7)-1)) << 25;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<4; ++i,++pt) out << *pt;

};

  void RhodamineData::unpack(boost::archive::binary_iarchive& in) 
{
    uint32_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<4; ++i,++pt) in >> *pt;
  status_flag = static_cast<uint8_t>(storage & ((1<<1)-1));
  storage >>= 1;
  lat = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  lon = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  depth = static_cast<uint8_t>(storage & ((1<<4)-1));
  storage >>= 4;
  adc_gain = static_cast<uint8_t>(storage & ((1<<0)-1));
  storage >>= 0;
  adc = static_cast<uint16_t>(storage & ((1<<7)-1));
  storage >>= 7;

};




  void LupisUpdate::pack(boost::archive::binary_oarchive& out) const
{
    uint32_t storage = 0;
  storage |= (uint32_t(cmd_flag) & ((1<<4)-1)) << 0;
  storage |= (uint32_t(lat) & ((1<<10)-1)) << 4;
  storage |= (uint32_t(lon) & ((1<<10)-1)) << 14;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<3; ++i,++pt) out << *pt;

};

  void LupisUpdate::unpack(boost::archive::binary_iarchive& in) 
{
    uint32_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<3; ++i,++pt) in >> *pt;
  cmd_flag = static_cast<uint8_t>(storage & ((1<<4)-1));
  storage >>= 4;
  lat = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  lon = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;

};




