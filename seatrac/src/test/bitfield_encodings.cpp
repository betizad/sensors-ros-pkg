#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/serialization.h>
#include <labust/seatrac/detail/serialization_defs.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>

#include <bitset>

int main(int argc, char* argv[])
{
	using namespace labust::seatrac;
	using namespace boost::iostreams;
	BitMessageTest test;
	test.lat = 1024;
	test.lon = 4194303;
	test.type = 4;
  std::bitset<32> a((1<<22)-1);
	std::cout<<"Mask:"<<a<<std::endl;

	SeatracMessage::DataBuffer out;
	typedef back_insert_device<SeatracMessage::DataBuffer> smsink;
	smsink sink(out);
	stream<smsink> os(sink);
	boost::archive::binary_oarchive outser(os, boost::archive::no_header);
	test.pack(outser);
	os.flush();

	std::cout<<"Data ("<<out.size()<<"):";
	for(int i=0; i<out.size(); ++i)
	{
		std::cout<<std::hex<<int(out[i]);
	}
	std::cout<<std::endl;

  array_source source(out.data(), out.size());
  stream<array_source> is(source);
	boost::archive::binary_iarchive inser(is, boost::archive::no_header);
	BitMessageTest test2;
	test2.unpack(inser);

	std::cout<<"Lat:"<<std::dec<<int32_t(test2.lat)<<", lon:"<<int32_t(test2.lon)<<", type:"<<int(test2.type)<<std::endl;


  return 0;
}
