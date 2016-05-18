#include <iostream>
#include <labust/tools/latlon_encoder.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/tools/packer.h>

#include <vector>

using namespace labust::comms::caddy;
int main(int argc, char* argv[])
{
  DiverReport report;

  //report.is_master = false;
  /*report.inited = true;
  report.has_position = true;
  report.origin_lat = 0;
  report.origin_lon = 0;
  report.north = 10;
  report.east = 12;*/
  report.depth = 0;
  report.heading = 0;
  /*report.altitude = 3;
  report.course = 100;
  report.speed = 0.2;*/

  std::vector<char> buf;
  labust::tools::encodePackable(report,&buf);


  std::cout << "encoded:";
  for (int i=0; i < buf.size(); ++i)
    std::cout << std::hex << uint16_t(buf[i]) << " ";
  std::cout<<"\n";

  DiverReport report2;
  labust::tools::decodePackable(buf, &report2);

  std::cout << "Report:\n";
  //std::cout<<"\t inited:"<<int(report2.inited)<<std::endl;
  //std::cout<<"\t has_position:"<<int(report2.has_position)<<std::endl;
  //std::cout<<"\t north:"<<report2.north<<std::endl;
  //std::cout<<"\t east:"<<report2.east<<std::endl;
  std::cout<<"\t depth:"<<report2.depth<<std::endl;
  //std::cout<<"\t altitude:"<<report2.altitude<<std::endl;
  //std::cout<<"\t course:"<<report2.course<<std::endl;
  //std::cout<<"\t speed:"<<report2.speed<<std::endl;


  return 0;
}
