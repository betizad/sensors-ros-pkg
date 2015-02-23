
bool StatusResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool StatusResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingSendResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingSendResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingReq::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingReq::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool PingError::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool PingError::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatSendResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatSendResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatReceive::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatReceive::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatError::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatError::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueSetResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueSetResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueClearResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueClearResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}



bool DatQueueStatusResp::pack(SeatracMessage::DataBuffer& out) const
{
  return seatrac_serialize(this, out);
}

bool DatQueueStatusResp::unpack(const SeatracMessage::DataBuffer& in)
{
  return seatrac_deserialize(this, in);
}


