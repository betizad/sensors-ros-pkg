
bool PingSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DataSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DataSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool StatusCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool StatusCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}


