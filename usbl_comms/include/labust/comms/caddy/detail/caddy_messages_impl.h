  void BuddyReport::pack(boost::archive::binary_oarchive& out) const
{
    uint64_t storage = 0;
  storage |= (uint64_t(msg_id) & ((1<<3)-1)) << 0;
  storage |= (uint64_t(offset_x) & ((1<<10)-1)) << 3;
  storage |= (uint64_t(offset_y) & ((1<<10)-1)) << 13;
  storage |= (uint64_t(course) & ((1<<10)-1)) << 23;
  storage |= (uint64_t(speed) & ((1<<4)-1)) << 33;
  storage |= (uint64_t(depth) & ((1<<7)-1)) << 37;
  storage |= (uint64_t(diver_offset_x) & ((1<<10)-1)) << 44;
  storage |= (uint64_t(diver_offset_y) & ((1<<10)-1)) << 54;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<8; ++i,++pt) out << *pt;

};

  void BuddyReport::unpack(boost::archive::binary_iarchive& in) 
{
    uint64_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<8; ++i,++pt) in >> *pt;
  msg_id = static_cast<uint8_t>(storage & ((1<<3)-1));
  storage >>= 3;
  offset_x = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  offset_y = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  course = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  speed = static_cast<uint8_t>(storage & ((1<<4)-1));
  storage >>= 4;
  depth = static_cast<uint8_t>(storage & ((1<<7)-1));
  storage >>= 7;
  diver_offset_x = static_cast<uint8_t>(storage & ((1<<10)-1));
  storage >>= 10;
  diver_offset_y = static_cast<uint8_t>(storage & ((1<<10)-1));
  storage >>= 10;

};




  void SurfaceNav::pack(boost::archive::binary_oarchive& out) const
{
    uint64_t storage = 0;
  storage |= (uint64_t(msg_id) & ((1<<3)-1)) << 0;
  storage |= (uint64_t(course) & ((1<<10)-1)) << 3;
  storage |= (uint64_t(speed) & ((1<<4)-1)) << 13;
  storage |= (uint64_t(offset_x) & ((1<<10)-1)) << 17;
  storage |= (uint64_t(offset_y) & ((1<<10)-1)) << 27;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<5; ++i,++pt) out << *pt;

};

  void SurfaceNav::unpack(boost::archive::binary_iarchive& in) 
{
    uint64_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<5; ++i,++pt) in >> *pt;
  msg_id = static_cast<uint8_t>(storage & ((1<<3)-1));
  storage >>= 3;
  course = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  speed = static_cast<int32_t>(storage & ((1<<4)-1));
  storage >>= 4;
  offset_x = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  offset_y = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;

};




  void DiverNav::pack(boost::archive::binary_oarchive& out) const
{
    uint32_t storage = 0;
  storage |= (uint32_t(msg_id) & ((1<<3)-1)) << 0;
  storage |= (uint32_t(heading) & ((1<<10)-1)) << 3;
  storage |= (uint32_t(depth) & ((1<<7)-1)) << 13;
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<3; ++i,++pt) out << *pt;

};

  void DiverNav::unpack(boost::archive::binary_iarchive& in) 
{
    uint32_t storage(0);
  uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));
  for(int i=0; i<3; ++i,++pt) in >> *pt;
  msg_id = static_cast<uint8_t>(storage & ((1<<3)-1));
  storage >>= 3;
  heading = static_cast<int32_t>(storage & ((1<<10)-1));
  storage >>= 10;
  depth = static_cast<int32_t>(storage & ((1<<7)-1));
  storage >>= 7;

};




