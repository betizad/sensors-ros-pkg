  void RhodamineData::pack(boost::archive::binary_oarchive& out) const
{
    uint64_t storage = 0;
  storage |= (uint64_t(status_flag) & ((1<<8)-1)) << 0;
  storage |= (uint64_t(lat) & ((1<<14)-1)) << 8;
  storage |= (uint64_t(lon) & ((1<<14)-1)) << 22;
  storage |= (uint64_t(depth) & ((1<<8)-1)) << 36;
  storage |= (uint64_t(adc_gain) & ((1<<2)-1)) << 44;
  storage |= (uint64_t(adc) & ((1<<10)-1)) << 46;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<7; ++i,++pt) out << *pt;

};

  void RhodamineData::unpack(boost::archive::binary_iarchive& in) 
{
    uint64_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<7; ++i,++pt) in >> *pt;
  status_flag = static_cast<uint8_t>(storage & ((1<<8)-1));
  storage >>= 8;
  lat = static_cast<int32_t>(storage & ((1<<14)-1));
  storage >>= 14;
  lon = static_cast<int32_t>(storage & ((1<<14)-1));
  storage >>= 14;
  depth = static_cast<uint8_t>(storage & ((1<<8)-1));
  storage >>= 8;
  adc_gain = static_cast<uint8_t>(storage & ((1<<2)-1));
  storage >>= 2;
  adc = static_cast<uint16_t>(storage & ((1<<10)-1));
  storage >>= 10;

};




  void LupisUpdate::pack(boost::archive::binary_oarchive& out) const
{
    uint64_t storage = 0;
  storage |= (uint64_t(cmd_flag) & ((1<<4)-1)) << 0;
  storage |= (uint64_t(lat) & ((1<<18)-1)) << 4;
  storage |= (uint64_t(lon) & ((1<<18)-1)) << 22;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<5; ++i,++pt) out << *pt;

};

  void LupisUpdate::unpack(boost::archive::binary_iarchive& in) 
{
    uint64_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<5; ++i,++pt) in >> *pt;
  cmd_flag = static_cast<uint8_t>(storage & ((1<<4)-1));
  storage >>= 4;
  lat = static_cast<int32_t>(storage & ((1<<18)-1));
  storage >>= 18;
  lon = static_cast<int32_t>(storage & ((1<<18)-1));
  storage >>= 18;

};




