  void BuddyReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(inited,0,1,1);
  if ((inited != 1)){
  st.put(origin_lat,-90,90,31);
  }
  if ((inited != 1)){
  st.put(origin_lon,-180,180,31);
  }
  if ((inited == 1)){
  st.put(has_position,0,1,1);
  }
  if ((inited == 1) && (has_position == 1)){
  st.put(north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  st.put(east,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  st.put(depth,0,63.5,7);
  }
  if ((inited == 1) && (has_position == 1)){
  st.put(altitude,0,7.75,5);
  }
  if ((inited == 1)){
  st.put(course,-180,180,9);
  }
  if ((inited == 1)){
  st.put(speed,0,1,4);
  }
  if ((inited == 1)){
  st.put(has_diver,0,1,1);
  }
  if ((inited == 1) && (has_diver == 1)){
  st.put(diver_north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_diver == 1)){
  st.put(diver_east,-51.2,51.1,10);
  }
  if ((inited == 1)){
  st.put(battery_status,0,100,3);
  }
  if ((inited == 1)){
  st.put(leak_info,0,1,1);
  }
  if ((inited == 1)){
  st.put(command,0,7,3);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  st.put(north_origin,-32,31,6);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  st.put(east_origin,-32,31,6);
  }
  if ((inited == 1) && (command == 1)){
  st.put(lawn_width,1,9,3);
  }
  if ((inited == 1) && (command == 1)){
  st.put(lawn_length,1,9,3);
  }
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void BuddyReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  try{
  while(true)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  } catch (std::exception& e){};
  labust::tools::BitStorage st(data);
  st.get(inited,0,1,1);
  if ((inited != 1)){
  st.get(origin_lat,-90,90,31);
  }
  if ((inited != 1)){
  st.get(origin_lon,-180,180,31);
  }
  if ((inited == 1)){
  st.get(has_position,0,1,1);
  }
  if ((inited == 1) && (has_position == 1)){
  st.get(north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  st.get(east,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  st.get(depth,0,63.5,7);
  }
  if ((inited == 1) && (has_position == 1)){
  st.get(altitude,0,7.75,5);
  }
  if ((inited == 1)){
  st.get(course,-180,180,9);
  }
  if ((inited == 1)){
  st.get(speed,0,1,4);
  }
  if ((inited == 1)){
  st.get(has_diver,0,1,1);
  }
  if ((inited == 1) && (has_diver == 1)){
  st.get(diver_north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_diver == 1)){
  st.get(diver_east,-51.2,51.1,10);
  }
  if ((inited == 1)){
  st.get(battery_status,0,100,3);
  }
  if ((inited == 1)){
  st.get(leak_info,0,1,1);
  }
  if ((inited == 1)){
  st.get(command,0,7,3);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  st.get(north_origin,-32,31,6);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  st.get(east_origin,-32,31,6);
  }
  if ((inited == 1) && (command == 1)){
  st.get(lawn_width,1,9,3);
  }
  if ((inited == 1) && (command == 1)){
  st.get(lawn_length,1,9,3);
  }

};




  void SurfaceReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(is_master,0,1,1);
  if ((is_master == 1)){
  st.put(inited,0,1,1);
  }
  if (((is_master == 1) && (inited != 1))){
  st.put(origin_lat,-90,90,31);
  }
  if (((is_master == 1) && (inited != 1))){
  st.put(origin_lon,-180,180,31);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.put(north,-51.2,51.1,10);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.put(east,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1))){
  st.put(has_diver,0,1,1);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  st.put(diver_north,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  st.put(diver_east,-51.2,51.1,10);
  }
  if ((is_master == 0)){
  st.put(course,-180,180,9);
  }
  if ((is_master == 0)){
  st.put(speed,0,1.0,4);
  }
  if ((is_master == 0)){
  st.put(command,0,7,3);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  st.put(north_origin,-32,31,6);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  st.put(east_origin,-32,31,6);
  }
  if ((is_master == 0) && (command == 1)){
  st.put(lawn_width,1,9,3);
  }
  if ((is_master == 0) && (command == 1)){
  st.put(lawn_length,1,9,3);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.put(predefined_chat,0,31,5);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  for(int i=0; i<chat.size(); ++i) st.put(chat[i],0,63,6);
  }
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void SurfaceReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  try{
  while(true)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  } catch (std::exception& e){};
  labust::tools::BitStorage st(data);
  st.get(is_master,0,1,1);
  if ((is_master == 1)){
  st.get(inited,0,1,1);
  }
  if (((is_master == 1) && (inited != 1))){
  st.get(origin_lat,-90,90,31);
  }
  if (((is_master == 1) && (inited != 1))){
  st.get(origin_lon,-180,180,31);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.get(north,-51.2,51.1,10);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.get(east,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1))){
  st.get(has_diver,0,1,1);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  st.get(diver_north,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  st.get(diver_east,-51.2,51.1,10);
  }
  if ((is_master == 0)){
  st.get(course,-180,180,9);
  }
  if ((is_master == 0)){
  st.get(speed,0,1.0,4);
  }
  if ((is_master == 0)){
  st.get(command,0,7,3);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  st.get(north_origin,-32,31,6);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  st.get(east_origin,-32,31,6);
  }
  if ((is_master == 0) && (command == 1)){
  st.get(lawn_width,1,9,3);
  }
  if ((is_master == 0) && (command == 1)){
  st.get(lawn_length,1,9,3);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  st.get(predefined_chat,0,31,5);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  chat.resize(st.remaining());
  for(int i=0; i<st.remaining(); ++i) st.get(chat[i],0,63,6);
  }

};




  void DiverReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  st.put(heading,-180,180,9);
  st.put(depth,0,63.5,7);
  st.put(avg_flipper_rate,0,15,4);
  st.put(hearth_rate,0,150,8);
  st.put(optional_data,0,1,1);
  st.put(alarms,0,7,3);
  st.put(predefined_chat,0,31,5);
  st.put(command,0,7,3);
  if (((command == 1) || (command == 2) || (command == 4))){
  st.put(north_origin,-32,31,6);
  }
  if (((command == 1) || (command == 2) || (command == 4))){
  st.put(east_origin,-32,31,6);
  }
  if ((command == 1)){
  st.put(lawn_width,1,9,3);
  }
  if ((command == 1)){
  st.put(lawn_length,1,9,3);
  }
  for(int i=0; i<chat.size(); ++i) st.put(chat[i],0,63,6);
  if ((optional_data == 1)){
  st.put(breathing_rate,0,30,7);
  }
  if ((optional_data == 1)){
  st.put(motion_rate,0,3,2);
  }
  if ((optional_data == 1)){
  st.put(pad_space,0,31,5);
  }
  for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];

};

  void DiverReport::unpack(boost::archive::binary_iarchive& in) 
{
    std::vector<uint8_t> data;
  try{
  while(true)
  {
  uint8_t temp;
  in >> temp;
  data.push_back(temp);
  }
  } catch (std::exception& e){};
  labust::tools::BitStorage st(data);
  st.get(heading,-180,180,9);
  st.get(depth,0,63.5,7);
  st.get(avg_flipper_rate,0,15,4);
  st.get(hearth_rate,0,150,8);
  st.get(optional_data,0,1,1);
  st.get(alarms,0,7,3);
  st.get(predefined_chat,0,31,5);
  st.get(command,0,7,3);
  if (((command == 1) || (command == 2) || (command == 4))){
  st.get(north_origin,-32,31,6);
  }
  if (((command == 1) || (command == 2) || (command == 4))){
  st.get(east_origin,-32,31,6);
  }
  if ((command == 1)){
  st.get(lawn_width,1,9,3);
  }
  if ((command == 1)){
  st.get(lawn_length,1,9,3);
  }
  chat.resize(st.remaining());
  for(int i=0; i<st.remaining(); ++i) st.get(chat[i],0,63,6);
  if ((optional_data == 1)){
  st.get(breathing_rate,0,30,7);
  }
  if ((optional_data == 1)){
  st.get(motion_rate,0,3,2);
  }
  if ((optional_data == 1)){
  st.get(pad_space,0,31,5);
  }

};




