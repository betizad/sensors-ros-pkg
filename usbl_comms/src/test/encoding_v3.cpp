#include <iostream>
#include <labust/tools/latlon_encoder.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/tools/packer.h>

#include <vector>

using labust::comms::caddy::BuddyReport;
using labust::comms::caddy::SurfaceReport;
using labust::comms::caddy::DiverReport;
enum
{
  NO_CHANGE = 0,
  LAWN_CMD = 1,
  GUIDE_ME = 2,
  GET_TOOL = 3,
  TAKE_PHOTO = 4,
  FAILED_CMD = 6,
  STOP = 7
};

void fillReport(DiverReport& report)
{
  report.heading = 45;
  report.depth = 14;
  report.command = 1;
  report.north_origin = 30;
  report.east_origin = 22;
  report.lawn_width = 2;
  report.lawn_length = 3;

  report.avg_flipper_rate = 2*3/16.;
  report.hearth_rate = 110*150/256.;
  report.breathing_rate = 5*30/128.;
  report.motion_rate = 2;
  report.pad_space = 25;
  report.predefined_chat = 5;
  report.alarms = 3;
}

void fillReport(SurfaceReport& report)
{
  report.origin_lat = 42.123456789;
  report.origin_lon = 15.123456789;
  report.north = 10.1;
  report.east = -12.5;
  report.course = 45;
  report.speed = 0.250;
  report.diver_north = 22.3;
  report.diver_east = 1.4;
  report.command = 1;
  report.north_origin = 30;
  report.east_origin = 22;
  report.lawn_width = 2;
  report.lawn_length = 3;
  report.predefined_chat = 10;
}

void fillReport(BuddyReport& report)
{
  report.origin_lat = 42.123456789;
  report.origin_lon = 15.123456789;
  report.north = 10.1;
  report.east = -12.5;
  report.depth = 14.0;
  report.altitude = 3.25;
  report.course = 45;
  report.speed = 0.250;
  report.diver_north = 22.3;
  report.diver_east = 1.4;
  report.battery_status = 42;
  report.leak_info = 1;
  report.command = 1;
  report.north_origin = 30;
  report.east_origin = 22;
  report.lawn_width = 2;
  report.lawn_length = 3;
}

void printReport(const BuddyReport& report)
{
  std::cout<<"\t inited:"<<int(report.inited)<<std::endl;
  if (!report.inited)
  {
    std::cout.precision(10);
    std::cout<<"\t lat:"<<report.origin_lat<<std::endl;
    std::cout<<"\t lon:"<<report.origin_lon<<std::endl;
    std::cout.precision(6);
    return;
  }

  std::cout<<"\t has_position:"<<int(report.has_position)<<std::endl;
  if (report.has_position)
  {
    std::cout<<"\t north:"<<report.north<<std::endl;
    std::cout<<"\t east:"<<report.east<<std::endl;
    std::cout<<"\t depth:"<<report.depth<<std::endl;
    std::cout<<"\t altitude:"<<report.altitude<<std::endl;
  }
  std::cout<<"\t course:"<<report.course<<std::endl;
  std::cout<<"\t speed:"<<report.speed<<std::endl;
  
  std::cout<<"\t has_diver:"<<int(report.has_diver)<<std::endl;
  if (report.has_diver)
  {
    std::cout<<"\t diver_north:"<<report.diver_north<<std::endl;
    std::cout<<"\t diver_east:"<<report.diver_east<<std::endl;
  }

  std::cout<<"\t battery_status:"<<int(report.battery_status)<<std::endl;
  std::cout<<"\t leak_info:"<<int(report.leak_info)<<std::endl;
  std::cout<<"\t command:"<<int(report.command)<<std::endl;

  switch(report.command)
  {
    case GUIDE_ME:
    case TAKE_PHOTO:
      std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
      std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
      break;
    case LAWN_CMD:
      std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
      std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
      std::cout<<"\t lawn_width:"<<report.lawn_width<<std::endl;
      std::cout<<"\t lawn_length:"<<report.lawn_length<<std::endl;
      break;
    default: break;
  }
}

void printReport(const SurfaceReport& report)
{
  std::cout<<"\t inited:"<<int(report.inited)<<std::endl;
  if (report.is_master && !report.inited)
  {
    std::cout.precision(10);
    std::cout<<"\t lat:"<<report.origin_lat<<std::endl;
    std::cout<<"\t lon:"<<report.origin_lon<<std::endl;
    std::cout.precision(6);
    return;
  }
  
  std::cout<<"\t is_master:"<<int(report.is_master)<<std::endl;
  std::cout<<"\t north:"<<report.north<<std::endl;
  std::cout<<"\t east:"<<report.east<<std::endl;
  
  if (!report.is_master)
  {
    std::cout<<"\t course:"<<report.course<<std::endl;
    std::cout<<"\t speed:"<<report.speed<<std::endl;
    std::cout<<"\t command:"<<int(report.command)<<std::endl;

    switch(report.command)
    {
      case GUIDE_ME:
      case TAKE_PHOTO:
        std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
        std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
        break;
      case LAWN_CMD:
        std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
        std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
        std::cout<<"\t lawn_width:"<<report.lawn_width<<std::endl;
        std::cout<<"\t lawn_length:"<<report.lawn_length<<std::endl;
        break;
      default: break;
    }
  }

  if (report.is_master && report.inited)
  {
    std::cout<<"\t has_diver:"<<int(report.has_diver)<<std::endl;
    if (report.has_diver)
    {
      std::cout<<"\t diver_north:"<<report.diver_north<<std::endl;
      std::cout<<"\t diver_east:"<<report.diver_east<<std::endl;
    }  
  }

  std::cout<<"\t predefined_chat:"<<int(report.predefined_chat)<<std::endl;
}

void printReport(const DiverReport& report)
{
  std::cout<<"\t course:"<<report.heading<<std::endl;
  std::cout<<"\t depth:"<<report.depth<<std::endl;

  std::cout<<"\t avg_flipper_rate:"<<report.avg_flipper_rate<<std::endl;
  std::cout<<"\t hearth_rate:"<<report.hearth_rate<<std::endl;
  std::cout<<"\t optional_data:"<<int(report.optional_data)<<std::endl;

  if (report.optional_data)
  {
    std::cout<<"\t breathing_rate:"<<report.breathing_rate<<std::endl;
    std::cout<<"\t motion_rate:"<<report.motion_rate<<std::endl;
    std::cout<<"\t pad_space:"<<report.pad_space<<std::endl;
  }

  std::cout<<"\t alarms:"<<int(report.alarms)<<std::endl;
  std::cout<<"\t command:"<<int(report.command)<<std::endl;

  switch(report.command)
  {
    case GUIDE_ME:
    case TAKE_PHOTO:
      std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
      std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
      break;
    case LAWN_CMD:
      std::cout<<"\t north_origin:"<<report.north_origin<<std::endl;
      std::cout<<"\t east_origin:"<<report.east_origin<<std::endl;
      std::cout<<"\t lawn_width:"<<report.lawn_width<<std::endl;
      std::cout<<"\t lawn_length:"<<report.lawn_length<<std::endl;
      break;
    default: break;
  }

  std::cout<<"\t predefined_chat:"<<int(report.predefined_chat)<<std::endl;
}

template <class ReportType>
void encodeDecodeAndPrintReport(const ReportType& report)
{
  // Encode
  std::vector<char> buf;
  labust::tools::encodePackable(report,&buf);
  std::cout<< "ENCODED SIZE:" << buf.size() << std::endl;
  ReportType report2;
  labust::tools::decodePackable(buf, &report2);
  printReport(report2);
}


void testBuddyReportPosition(BuddyReport& report)
{
  // Set as Inited and having position 
  report.inited = true;
  report.has_position = true;
  report.has_diver = false;
  // Decode and print
  std::cout << "Buddy report with position:\n";
  encodeDecodeAndPrintReport(report);
}

void testBuddyReportNoPosition(BuddyReport& report)
{
  // Set as Inited and having position 
  report.inited = true;
  report.has_position = false;
  report.has_diver = false;
  // Decode and print
  std::cout << "Buddy report with no position:\n";
  encodeDecodeAndPrintReport(report);
}

void testBuddyReportNoPositionDiver(BuddyReport& report)
{
  // Set as Inited and having position 
  report.inited = true;
  report.has_position = false;
  report.has_diver = true;
  // Decode and print
  std::cout << "Buddy report with no position and diver:\n";
  encodeDecodeAndPrintReport(report);
}

void testBuddyReportPositionDiver(BuddyReport& report)
{
  // Set as Inited and having position 
  report.inited = true;
  report.has_position = true;
  report.has_diver = true;
  // Decode and print
  std::cout << "Buddy report with position and diver:\n";
  encodeDecodeAndPrintReport(report);
}

void testBuddyReportInit(BuddyReport& report)
{
  // Set as Inited and having position 
  report.inited = false;
  report.has_position = true;
  report.has_diver = true;
  // Decode and print
  std::cout << "Buddy report with Init position:\n";
  encodeDecodeAndPrintReport(report);
}

void testDiverReportNoOptional(DiverReport& report)
{
  // Set as Inited and having position 
  report.optional_data = false;
  // Decode and print
  std::cout << "Diver report with no optional:\n";
  encodeDecodeAndPrintReport(report);
}

void testDiverReportOptional(DiverReport& report)
{
  // Set as Inited and having position 
  report.optional_data = true;
  // Decode and print
  std::cout << "Diver report with optional:\n";
  encodeDecodeAndPrintReport(report);
}

void testSurfaceReportInit(SurfaceReport& report)
{
  // Set as Inited and having position 
  report.is_master = true;
  report.inited = false;
  report.has_diver = false;
  // Decode and print
  std::cout << "Surface report with Init:\n";
  encodeDecodeAndPrintReport(report);
}

void testSurfaceReportNoMaster(SurfaceReport& report)
{
  // Set as Inited and having position 
  report.is_master = false;
  report.inited = true;
  report.has_diver = false;
  // Decode and print
  std::cout << "Surface report with no master:\n";
  encodeDecodeAndPrintReport(report);
}

void testSurfaceReportMasterNoDiver(SurfaceReport& report)
{
  // Set as Inited and having position 
  report.is_master = true;
  report.inited = true;
  report.inited = true;
  report.is_master = true;
  report.has_diver = true;
  // Decode and print
  std::cout << "Surface report with master no diver:\n";
  encodeDecodeAndPrintReport(report);
}

void testSurfaceReportMasterDiver(SurfaceReport& report)
{
  // Set as Inited and having position 
  report.inited = true;
  report.is_master = true;
  report.has_diver = true;
  // Decode and print
  std::cout << "Surface report with master and diver:\n";
  encodeDecodeAndPrintReport(report);
}

using namespace labust::comms::caddy;
int main(int argc, char* argv[])
{
  // Test example for the Complete report
  /*BuddyReport breport;
  fillReport(breport);
  breport.inited = 0;
  breport.has_position = 1;
  breport.has_diver = 1;
  std::cout << "Complete Buddy Report used for encoding:\n";
  printReport(breport);
  breport.inited = 1;
  printReport(breport);
  // Test Buddy Report
  //testBuddyReportInit(breport);
  breport.command = 0;
  testBuddyReportNoPosition(breport);
  //testBuddyReportPosition(breport);
  //testBuddyReportNoPositionDiver(breport);
  //testBuddyReportPositionDiver(breport);
 /
  // Test example for the Complete report
  DiverReport dreport;
  fillReport(dreport);
  dreport.optional_data = 1;
  std::cout << "Complete Diver Report used for encoding:\n";
  printReport(dreport);
  // Test Diver Report
  //testDiverReportNoOptional(dreport);
  testDiverReportOptional(dreport);
  */

  // Test example for the Complete report
  SurfaceReport sreport;
  fillReport(sreport);
  sreport.inited = 0;
  sreport.is_master = 1;
  sreport.has_diver = 1;
  std::cout << "Complete Buddy Report used for encoding:\n";
  printReport(sreport);
  sreport.inited = 1;
  printReport(sreport);
  // Test Surface Report
  //testSurfaceReportInit(sreport);
  //testSurfaceReportNoMaster(sreport);
  testSurfaceReportMasterNoDiver(sreport);
  //testSurfaceReportMasterDiver(sreport);

  return 0;
}
