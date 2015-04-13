
bool StatusResp::isCommand() const
{
  return false;
}

bool StatusResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool StatusResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingSendResp::isCommand() const
{
  return false;
}

bool PingSendResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingSendResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingReq::isCommand() const
{
  return false;
}

bool PingReq::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingReq::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingResp::isCommand() const
{
  return false;
}

bool PingResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingError::isCommand() const
{
  return false;
}

bool PingError::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingError::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatSendResp::isCommand() const
{
  return false;
}

bool DatSendResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatSendResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatReceive::isCommand() const
{
  return false;
}

bool DatReceive::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatReceive::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatError::isCommand() const
{
  return false;
}

bool DatError::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatError::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueSetResp::isCommand() const
{
  return false;
}

bool DatQueueSetResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueSetResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueClearResp::isCommand() const
{
  return false;
}

bool DatQueueClearResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueClearResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueStatusResp::isCommand() const
{
  return false;
}

bool DatQueueStatusResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueStatusResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}


