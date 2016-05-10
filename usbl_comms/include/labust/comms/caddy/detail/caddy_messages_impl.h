  void BuddyReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(offset_x,-51.2,51.1,10);
  st.put(offset_y,-51.2,51.1,10);
  st.put(course,-180,180,9);
  st.put(speed,0,1,4);
  st.put(depth,0,63.5,7);
  st.put(altitude,0,7.75,5);
  st.put(battery_info,0,100,3);
  st.put(leak_info,0,1,1);
  st.put(mission_status,0,3,2);
  st.put(diver_offset_x,-51.2,51.2,10);
  st.put(diver_offset_y,-51.2,51.2,10);
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void BuddyReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  for(int i=0; i<9; ++i)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  labust::tools::BitStorage st(data);
  st.get(offset_x,-51.2,51.1,10);
  st.get(offset_y,-51.2,51.1,10);
  st.get(course,-180,180,9);
  st.get(speed,0,1,4);
  st.get(depth,0,63.5,7);
  st.get(altitude,0,7.75,5);
  st.get(battery_info,0,100,3);
  st.get(leak_info,0,1,1);
  st.get(mission_status,0,3,2);
  st.get(diver_offset_x,-51.2,51.2,10);
  st.get(diver_offset_y,-51.2,51.2,10);

};




  void SurfaceReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(offset_x,-51.2,51.1,10);
  st.put(offset_y,-51.2,51.1,10);
  st.put(course,-180,180,9);
  st.put(speed,0,1.0,4);
  st.put(mission_cmd,0,3,2);
  st.put(lawn_width,0,15,4);
  st.put(lawn_length,0,15,4);
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void SurfaceReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  for(int i=0; i<6; ++i)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  labust::tools::BitStorage st(data);
  st.get(offset_x,-51.2,51.1,10);
  st.get(offset_y,-51.2,51.1,10);
  st.get(course,-180,180,9);
  st.get(speed,0,1.0,4);
  st.get(mission_cmd,0,3,2);
  st.get(lawn_width,0,15,4);
  st.get(lawn_length,0,15,4);

};




  void SurfaceChat::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(offset_x,-51.2,51.1,10);
  st.put(offset_y,-51.2,51.1,10);
  st.put(course,-180,180,9);
  st.put(speed,0,1.0,4);
  for(int i=0; i<5; ++i) st.put(chat[i],0,63,6);
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void SurfaceChat::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  for(int i=0; i<8; ++i)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  labust::tools::BitStorage st(data);
  st.get(offset_x,-51.2,51.1,10);
  st.get(offset_y,-51.2,51.1,10);
  st.get(course,-180,180,9);
  st.get(speed,0,1.0,4);
  for(int i=0; i<5; ++i) st.get(chat[i],0,63,6);

};




  void DiverReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(heading,-180,180,9);
  st.put(depth,0,63.5,7);
  st.put(paddle_rate,0,15,4);
  st.put(hearth_rate,0,15,4);
  st.put(mission_cmd,0,3,2);
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void DiverReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  for(int i=0; i<4; ++i)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  labust::tools::BitStorage st(data);
  st.get(heading,-180,180,9);
  st.get(depth,0,63.5,7);
  st.get(paddle_rate,0,15,4);
  st.get(hearth_rate,0,15,4);
  st.get(mission_cmd,0,3,2);

};




  void DiverChat::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(paddle_rate,0,15,4);
  st.put(hearth_rate,0,15,4);
  for(int i=0; i<8; ++i) st.put(chat[i],0,63,6);
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void DiverChat::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  for(int i=0; i<7; ++i)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  labust::tools::BitStorage st(data);
  st.get(paddle_rate,0,15,4);
  st.get(hearth_rate,0,15,4);
  for(int i=0; i<8; ++i) st.get(chat[i],0,63,6);

};




