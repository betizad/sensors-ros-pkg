#include <iostream>
#include <labust/comms/uros/uros_messages.h>
#include <bitset>

using namespace labust::comms::uros;

int main(int argc, char* argv[])
{
	RhodamineReport a;
	a.rhodamine_adc = 800;
	a.adc_gain = 1;
	a.latitude = 42.12345678;
	a.longitude = 15.12345678;

	uint64_t out = a.encode();

	std::cout<<"Encoded:"<<std::bitset<64>(out)<<std::endl;

	RhodamineReport b;
	b.decode(out);

	std::cout<<"Before encoding: adc="<<a.rhodamine_adc<<", lat="<<a.latitude<<std::endl;
	std::cout<<"After encoding: adc="<<b.rhodamine_adc<<", lat="<<b.latitude<<std::endl;

	return 0;
}
