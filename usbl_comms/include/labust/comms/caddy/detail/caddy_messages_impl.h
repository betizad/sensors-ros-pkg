  void BuddyReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  std::cout<<"inited output:";  st.put(inited,0,1,1);
  if ((inited != 1)){
  std::cout<<"origin_lat output:";  st.put(origin_lat,-90,90,31);
  }
  if ((inited != 1)){
  std::cout<<"origin_lon output:";  st.put(origin_lon,-180,180,31);
  }
  if ((inited == 1)){
  std::cout<<"has_position output:";  st.put(has_position,0,1,1);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"north output:";  st.put(north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"east output:";  st.put(east,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"depth output:";  st.put(depth,0,63.5,7);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"altitude output:";  st.put(altitude,0,7.75,5);
  }
  if ((inited == 1)){
  std::cout<<"course output:";  st.put(course,-180,179.296875,9);
  }
  if ((inited == 1)){
  std::cout<<"speed output:";  st.put(speed,0,1,4);
  }
  if ((inited == 1)){
  std::cout<<"has_diver output:";  st.put(has_diver,0,1,1);
  }
  if ((inited == 1) && (has_diver == 1)){
  std::cout<<"diver_north output:";  st.put(diver_north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_diver == 1)){
  std::cout<<"diver_east output:";  st.put(diver_east,-51.2,51.1,10);
  }
  if ((inited == 1)){
  std::cout<<"battery_status output:";  st.put(battery_status,0,100,3);
  }
  if ((inited == 1)){
  std::cout<<"leak_info output:";  st.put(leak_info,0,1,1);
  }
  if ((inited == 1)){
  std::cout<<"command output:";  st.put(command,0,7,3);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin output:";  st.put(north_origin,-32,31,6);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin output:";  st.put(east_origin,-32,31,6);
  }
  if ((inited == 1) && (command == 1)){
  std::cout<<"lawn_width output:";  st.put(lawn_width,1,8,3);
  }
  if ((inited == 1) && (command == 1)){
  std::cout<<"lawn_length output:";  st.put(lawn_length,1,8,3);
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
  std::cout<<"inited input:";  st.get(inited,0,1,1);
  if ((inited != 1)){
  std::cout<<"origin_lat input:";  st.get(origin_lat,-90,90,31);
  }
  if ((inited != 1)){
  std::cout<<"origin_lon input:";  st.get(origin_lon,-180,180,31);
  }
  if ((inited == 1)){
  std::cout<<"has_position input:";  st.get(has_position,0,1,1);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"north input:";  st.get(north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"east input:";  st.get(east,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"depth input:";  st.get(depth,0,63.5,7);
  }
  if ((inited == 1) && (has_position == 1)){
  std::cout<<"altitude input:";  st.get(altitude,0,7.75,5);
  }
  if ((inited == 1)){
  std::cout<<"course input:";  st.get(course,-180,179.296875,9);
  }
  if ((inited == 1)){
  std::cout<<"speed input:";  st.get(speed,0,1,4);
  }
  if ((inited == 1)){
  std::cout<<"has_diver input:";  st.get(has_diver,0,1,1);
  }
  if ((inited == 1) && (has_diver == 1)){
  std::cout<<"diver_north input:";  st.get(diver_north,-51.2,51.1,10);
  }
  if ((inited == 1) && (has_diver == 1)){
  std::cout<<"diver_east input:";  st.get(diver_east,-51.2,51.1,10);
  }
  if ((inited == 1)){
  std::cout<<"battery_status input:";  st.get(battery_status,0,100,3);
  }
  if ((inited == 1)){
  std::cout<<"leak_info input:";  st.get(leak_info,0,1,1);
  }
  if ((inited == 1)){
  std::cout<<"command input:";  st.get(command,0,7,3);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin input:";  st.get(north_origin,-32,31,6);
  }
  if ((inited == 1) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin input:";  st.get(east_origin,-32,31,6);
  }
  if ((inited == 1) && (command == 1)){
  std::cout<<"lawn_width input:";  st.get(lawn_width,1,8,3);
  }
  if ((inited == 1) && (command == 1)){
  std::cout<<"lawn_length input:";  st.get(lawn_length,1,8,3);
  }

};




  void SurfaceReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  std::cout<<"is_master output:";  st.put(is_master,0,1,1);
  if ((is_master == 1)){
  std::cout<<"inited output:";  st.put(inited,0,1,1);
  }
  if (((is_master == 1) && (inited != 1))){
  std::cout<<"origin_lat output:";  st.put(origin_lat,-90,90,31);
  }
  if (((is_master == 1) && (inited != 1))){
  std::cout<<"origin_lon output:";  st.put(origin_lon,-180,180,31);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"north output:";  st.put(north,-51.2,51.1,10);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"east output:";  st.put(east,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1))){
  std::cout<<"has_diver output:";  st.put(has_diver,0,1,1);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  std::cout<<"diver_north output:";  st.put(diver_north,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  std::cout<<"diver_east output:";  st.put(diver_east,-51.2,51.1,10);
  }
  if ((is_master == 0)){
  std::cout<<"course output:";  st.put(course,-180,179.296875,9);
  }
  if ((is_master == 0)){
  std::cout<<"speed output:";  st.put(speed,0,1.0,4);
  }
  if ((is_master == 0)){
  std::cout<<"command output:";  st.put(command,0,7,3);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin output:";  st.put(north_origin,-32,31,6);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin output:";  st.put(east_origin,-32,31,6);
  }
  if ((is_master == 0) && (command == 1)){
  std::cout<<"lawn_width output:";  st.put(lawn_width,1,8,3);
  }
  if ((is_master == 0) && (command == 1)){
  std::cout<<"lawn_length output:";  st.put(lawn_length,1,8,3);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"predefined_chat output:";  st.put(predefined_chat,0,31,5);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"chat output:";  for(int i=0; i<chat.size(); ++i) st.put(chat[i],0,63,6);
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
  std::cout<<"is_master input:";  st.get(is_master,0,1,1);
  if ((is_master == 1)){
  std::cout<<"inited input:";  st.get(inited,0,1,1);
  }
  if (((is_master == 1) && (inited != 1))){
  std::cout<<"origin_lat input:";  st.get(origin_lat,-90,90,31);
  }
  if (((is_master == 1) && (inited != 1))){
  std::cout<<"origin_lon input:";  st.get(origin_lon,-180,180,31);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"north input:";  st.get(north,-51.2,51.1,10);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"east input:";  st.get(east,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1))){
  std::cout<<"has_diver input:";  st.get(has_diver,0,1,1);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  std::cout<<"diver_north input:";  st.get(diver_north,-51.2,51.1,10);
  }
  if (((is_master == 1) && (inited == 1)) && (has_diver == 1)){
  std::cout<<"diver_east input:";  st.get(diver_east,-51.2,51.1,10);
  }
  if ((is_master == 0)){
  std::cout<<"course input:";  st.get(course,-180,179.296875,9);
  }
  if ((is_master == 0)){
  std::cout<<"speed input:";  st.get(speed,0,1.0,4);
  }
  if ((is_master == 0)){
  std::cout<<"command input:";  st.get(command,0,7,3);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin input:";  st.get(north_origin,-32,31,6);
  }
  if ((is_master == 0) && ((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin input:";  st.get(east_origin,-32,31,6);
  }
  if ((is_master == 0) && (command == 1)){
  std::cout<<"lawn_width input:";  st.get(lawn_width,1,8,3);
  }
  if ((is_master == 0) && (command == 1)){
  std::cout<<"lawn_length input:";  st.get(lawn_length,1,8,3);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"predefined_chat input:";  st.get(predefined_chat,0,31,5);
  }
  if (((is_master == 0) || ((is_master == 1) && (inited == 1)))){
  std::cout<<"chat input:";  chat.resize(st.remaining());
  for(int i=0; i<st.remaining(); ++i) st.get(chat[i],0,63,6);
  }

};




  void DiverReport::pack(boost::archive::binary_oarchive& out) const
{
    labust::tools::BitStorage st;
  std::cout<<"heading output:";  st.put(heading,-180,179.296875,9);
  std::cout<<"depth output:";  st.put(depth,0,63.5,7);
  std::cout<<"avg_flipper_rate output:";  st.put(avg_flipper_rate,0,3,4);
  std::cout<<"hearth_rate output:";  st.put(hearth_rate,0,150,8);
  std::cout<<"optional_data output:";  st.put(optional_data,0,1,1);
  if ((optional_data == 1)){
  std::cout<<"breathing_rate output:";  st.put(breathing_rate,0,30,7);
  }
  if ((optional_data == 1)){
  std::cout<<"motion_rate output:";  st.put(motion_rate,0,3,2);
  }
  if ((optional_data == 1)){
  std::cout<<"pad_space output:";  st.put(pad_space,0,31,5);
  }
  std::cout<<"alarms output:";  st.put(alarms,0,7,3);
  std::cout<<"predefined_chat output:";  st.put(predefined_chat,0,31,5);
  std::cout<<"command output:";  st.put(command,0,7,3);
  if (((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin output:";  st.put(north_origin,-32,31,6);
  }
  if (((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin output:";  st.put(east_origin,-32,31,6);
  }
  if ((command == 1)){
  std::cout<<"lawn_width output:";  st.put(lawn_width,1,8,3);
  }
  if ((command == 1)){
  std::cout<<"lawn_length output:";  st.put(lawn_length,1,8,3);
  }
  std::cout<<"chat output:";  for(int i=0; i<chat.size(); ++i) st.put(chat[i],0,63,6);
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
  std::cout<<"heading input:";  st.get(heading,-180,179.296875,9);
  std::cout<<"depth input:";  st.get(depth,0,63.5,7);
  std::cout<<"avg_flipper_rate input:";  st.get(avg_flipper_rate,0,3,4);
  std::cout<<"hearth_rate input:";  st.get(hearth_rate,0,150,8);
  std::cout<<"optional_data input:";  st.get(optional_data,0,1,1);
  if ((optional_data == 1)){
  std::cout<<"breathing_rate input:";  st.get(breathing_rate,0,30,7);
  }
  if ((optional_data == 1)){
  std::cout<<"motion_rate input:";  st.get(motion_rate,0,3,2);
  }
  if ((optional_data == 1)){
  std::cout<<"pad_space input:";  st.get(pad_space,0,31,5);
  }
  std::cout<<"alarms input:";  st.get(alarms,0,7,3);
  std::cout<<"predefined_chat input:";  st.get(predefined_chat,0,31,5);
  std::cout<<"command input:";  st.get(command,0,7,3);
  if (((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"north_origin input:";  st.get(north_origin,-32,31,6);
  }
  if (((command == 1) || (command == 2) || (command == 4))){
  std::cout<<"east_origin input:";  st.get(east_origin,-32,31,6);
  }
  if ((command == 1)){
  std::cout<<"lawn_width input:";  st.get(lawn_width,1,8,3);
  }
  if ((command == 1)){
  std::cout<<"lawn_length input:";  st.get(lawn_length,1,8,3);
  }
  std::cout<<"chat input:";  chat.resize(st.remaining());
  for(int i=0; i<st.remaining(); ++i) st.get(chat[i],0,63,6);

};




