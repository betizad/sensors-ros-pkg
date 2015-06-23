#include <iostream>
#include <labust/tools/latlon_encoder.h>

int main(int argc, char* argv[])
{
	using namespace labust::tools;
	int sz = 14;
	double lat = 37.999999;
	double lon = -1.000000;
	LatLon2Bits llbits;
	llbits.convert(lat, lon, sz);

	double latI = 37.9934666;
	double lonI = -1.0045222;

	Bits2LatLon bitsll;
	bitsll.setInitLatLon(latI, lonI);
	bitsll.convert(llbits.lat, llbits.lon, sz);

	std::cout<<"Latitude:"<<llbits.lat<<", longidute:"<<llbits.lon<<std::endl;
	std::cout.precision(8);
	std::cout<<"Latitude:"<<bitsll.latitude<<", longidute:"<<bitsll.longitude<<std::endl;
	/*RhodamineData a;
	a.adc = 800;
	a.adc_gain = 1;
	a.lat = 42.12345678;
	a.lon = 15.12345678;

	uint64_t out = a.encode();

	std::cout<<"Encoded:"<<std::bitset<64>(out)<<std::endl;

	RhodamineReport b;
	b.decode(out);

	std::cout<<"Before encoding: adc="<<a.rhodamine_adc<<", lat="<<a.latitude<<std::endl;
	std::cout<<"After encoding: adc="<<b.rhodamine_adc<<", lat="<<b.latitude<<std::endl;
*/
	return 0;
}
