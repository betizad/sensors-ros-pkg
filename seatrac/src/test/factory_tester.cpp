#include <labust/seatrac/seatrac_messages.h>
#include <labust/seatrac/seatrac_factory.h>
#include <labust/preprocessor/clean_serializator.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/export.hpp>
#include <labust/seatrac/mediator.h>
#include <list>
#include <boost/bind.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>

#include <boost/crc.hpp>

#include <labust/seatrac/detail/serialization_defs.h>

PP_LABUST_CLEAN_ARRAY_ISERIALIZATOR_IMPL(boost::archive::binary_iarchive)
PP_LABUST_CLEAN_ARRAY_OSERIALIZATOR_IMPL(boost::archive::binary_oarchive)

#include <sstream>

using namespace labust::seatrac;

void processPingCmd(const PingSendCmd& data)
{
	std::cout<<"Destination:"<<int(data.dest)<<std::endl;
}


namespace boost{ namespace serialization{
	template<class Archive>
	void serialize(Archive& ar, SeatracMessage& object, const unsigned int version){}
}}

int main(int argc, char* argv[])
{
	SeatracFactory fact;

	PingSendCmd test1;
	test1.dest = 1;
	test1.msg_type = 2;

	std::vector<char> cleanbuf;
	boost::iostreams::back_insert_device<std::vector<char> > sink(cleanbuf);
	boost::iostreams::stream<boost::iostreams::back_insert_device<std::vector<char> > > os(sink);
	boost::archive::binary_oarchive outSer(os, boost::archive::no_header);
	outSer<<test1;
	os.flush();

	std::cout<<"Data:("<<cleanbuf.size()<<")";
	for(int i=0; i<cleanbuf.size(); ++i)
	{
		std::cout<<int(uint8_t(cleanbuf.at(i)))<<" ";
	}
	std::cout<<std::endl;

	SeatracMessage* msgp = reinterpret_cast<SeatracMessage*>(new PingSendCmd());
	PingSendCmd* testp = reinterpret_cast<PingSendCmd*>(msgp);
	testp->dest = 2;
	testp->msg_type = 3;

	std::stringstream out;
	out.str("");
	boost::archive::binary_oarchive outSer2(out, boost::archive::no_header);
	outSer2.register_type<PingSendCmd>();
	outSer2<<msgp;

	std::cout<<"Data:";
	for(int i=0; i<out.str().size(); ++i)
	{
		std::cout<<int(uint8_t(out.str().at(i)))<<" ";
	}
	std::cout<<std::endl;

	SeatracMessage::Ptr msg1 = fact.createCommand(PingSendCmd::CID);
	boost::shared_ptr<PingSendCmd> test = boost::dynamic_pointer_cast<PingSendCmd>(msg1);
	test->dest = 1;
	test->msg_type = 2;
	std::cout<<"test:"<<msg1->getCid()<<std::endl;
	SeatracMessage::DataBuffer buf;
	test->pack(buf);

	std::cout<<"Data:";
	for(int i=0; i<buf.size(); ++i)
	{
		std::cout<<int(uint8_t(buf.at(i)))<<" ";
	}
	std::cout<<std::endl;

	SeatracMessage::Ptr msg = fact.createCommand(0x41);
	boost::shared_ptr<PingSendCmd> cmsg = boost::dynamic_pointer_cast<PingSendCmd>(msg);
	cmsg->dest = 10;

	typedef boost::function<void (const SeatracMessage::ConstPtr&)> Callback;
	std::map<int, std::list<Callback> > dispatcher;
	Mediator<PingSendCmd> med(boost::bind(processPingCmd,_1));
	dispatcher[0x40].push_back(getcallback(med));
	dispatcher[msg->getCid()].front()(msg);
  return 0;
}
