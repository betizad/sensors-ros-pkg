#include <iostream>
#include <labust/tools/latlon_encoder.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/tools/packer.h>

#include <vector>

using namespace labust::comms::caddy;
int main(int argc, char* argv[])
{
  SurfaceReport report;

  report.is_master = false;
  report.inited = true;
  //report.has_position = true;
  report.origin_lat = 0;
  report.origin_lon = 0;
  report.north = 0;
  report.east = 0;
  //report.depth = 2.2;
  //report.altitude = 3;
  report.course = 0;
  report.speed = 0;

  std::vector<char> buf;
  labust::tools::encodePackable(report,&buf);


  std::cout << "encoded:";
  for (int i=0; i < buf.size(); ++i)
    std::cout << std::hex << uint16_t(buf[i]) << " ";
  std::cout<<"\n";

  labust::tools::decodePackable(buf, &report);

  std::cout << "Report:\n";
  std::cout<<"\t inited:"<<int(report.inited)<<std::endl;
  //std::cout<<"\t has_position:"<<int(report.has_position)<<std::endl;
  std::cout<<"\t north:"<<report.north<<std::endl;
  std::cout<<"\t east:"<<report.east<<std::endl;
  //std::cout<<"\t depth:"<<report.depth<<std::endl;
  //std::cout<<"\t altitude:"<<report.altitude<<std::endl;
  std::cout<<"\t course:"<<report.course<<std::endl;
  std::cout<<"\t speed:"<<report.speed<<std::endl;


  return 0;
}
