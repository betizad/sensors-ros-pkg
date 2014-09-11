/*
 * serialization_test.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: dnad
 */
#include <labust/archive/asciihex_iarchive.hpp>

#include <cstdint>
#include <string>
#include <sstream>

std::string ex_data()
{
	return "$0282330000011B0301690E000000000000FF900301006901B7FAC5BFFF910301007A07750463A95DDE";
}

int main()
{
	uint16_t dec = 13330;
	std::string encdec("02");

	std::istringstream in(encdec);
	//in.width(2);
	//in.fill('0');
	uint8_t indec(0);
	labust::archive::asciihex_iarchive inser(in);
	inser>>indec;

	std::cout<<"Decoded value: "<<uint32_t(indec)<<" == "<<uint32_t(dec)<<std::endl;

  return 0;
}
