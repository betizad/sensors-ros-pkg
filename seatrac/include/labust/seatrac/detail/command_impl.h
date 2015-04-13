
bool StatusCmd::isCommand() const
{
  return true;
}

bool StatusCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool StatusCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingSendCmd::isCommand() const
{
  return true;
}

bool PingSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatSendCmd::isCommand() const
{
  return true;
}

bool DatSendCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatSendCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueSetCmd::isCommand() const
{
  return true;
}

bool DatQueueSetCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueSetCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueClearCmd::isCommand() const
{
  return true;
}

bool DatQueueClearCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueClearCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueStatusCmd::isCommand() const
{
  return true;
}

bool DatQueueStatusCmd::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueStatusCmd::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}


