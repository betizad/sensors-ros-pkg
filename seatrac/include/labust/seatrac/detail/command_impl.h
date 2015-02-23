
bool StatusCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool StatusCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueSetCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueSetCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueClearCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueClearCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueStatusCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueStatusCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}


