#include <gtest/gtest.h>
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/tools/packer.h>
#include <cstdint>

using labust::comms::caddy::BuddyReport;
using labust::comms::caddy::DiverReport;
using labust::comms::caddy::SurfaceReport;

/// Report validation macro
#define DOUBLE_TOLERANCE 0.00001
#define VD_REP(x) EXPECT_EQ(report_in.x, report_out.x)
#define VD_REPD(x) EXPECT_NEAR(report_in.x, report_out.x, DOUBLE_TOLERANCE)

/// TODO: Refactor enumerator
class CaddyCommsStructuresTest : public ::testing::Test
{
protected:
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
  virtual void SetUp()
  {
    configureDiverReport();
    configureSurfaceReport();
    configureBuddyReport();
  }

  virtual void TearDown()
  {
  }

  void configureDiverReport()
  {
    // Nav data
    diver.heading = 62.578125;
    diver.depth = 7.5;
    // Basic info
    diver.avg_flipper_rate = 0.8;
    diver.hearth_rate = 150;
    diver.alarms = 5;
    diver.predefined_chat = 16;
    // Diver command IDLE by default
    diver.command = 0;
    // Command payload data
    diver.north_origin = 12;
    diver.east_origin = -26;
    diver.lawn_width = 6;
    diver.lawn_length = 4;
    // Ignore optional data by default
    diver.optional_data = 0;
    diver.breathing_rate = 30;
    diver.motion_rate = 2;
    diver.pad_space = 21;
    // Additional chat
    // diver.chat.push_back(0);
  }

  void configureSurfaceReport()
  {
    surface.origin_lat = 42.123456789;
    surface.origin_lon = 15.123456789;
    surface.north = 10.1;
    surface.east = -12.5;
    surface.course = 45;
    surface.speed = 0.266667;
    surface.diver_north = 22.3;
    surface.diver_east = 1.4;
    surface.command = 1;
    surface.north_origin = 30;
    surface.east_origin = 22;
    surface.lawn_width = 2;
    surface.lawn_length = 3;
    surface.predefined_chat = 10;
  }

  void configureBuddyReport()
  {
    buddy.origin_lat = 42.123456789;
    buddy.origin_lon = 15.123456789;
    buddy.north = 10.1;
    buddy.east = -12.5;
    buddy.depth = 14.0;
    buddy.altitude = 3.25;
    buddy.course = 45;
    buddy.speed = 0.266667;
    buddy.diver_north = 22.3;
    buddy.diver_east = 1.4;
    buddy.battery_status = 42;
    buddy.leak_info = 1;
    buddy.command = 1;
    buddy.north_origin = 30;
    buddy.east_origin = 22;
    buddy.lawn_width = 2;
    buddy.lawn_length = 3;
  }

  template <class ReportType>
  void encodeAndDecodeReport(const ReportType& report_in,
                             ReportType& report_out)
  {
    // Encode
    std::vector<char> buf;
    labust::tools::encodePackable(report_in, &buf);
    labust::tools::decodePackable(buf, &report_out);
    validateReport(report_in, report_out);
  }

  template <class ReportType>
  void validateCommand(const ReportType& report_in,
                       const ReportType& report_out)
  {
    switch (report_out.command)
    {
      case GUIDE_ME:
      case TAKE_PHOTO:
        VD_REP(north_origin);
        VD_REP(east_origin);
        break;
      case LAWN_CMD:
        VD_REP(north_origin);
        VD_REP(east_origin);
        VD_REP(lawn_width);
        VD_REP(lawn_length);
        break;
      default:
        break;
    }
  }

  template <class ReportType>
  bool validateInit(const ReportType& report_in, const ReportType& report_out)
  {
    if (!report_out.inited)
    {
      VD_REPD(origin_lat);
      VD_REPD(origin_lon);
    }
    return report_out.inited;
  }

  void validateReport(const DiverReport& report_in,
                      const DiverReport& report_out)
  {
    // Nav data
    VD_REP(heading);
    VD_REP(depth);
    // Basic info
    VD_REP(avg_flipper_rate);
    VD_REP(hearth_rate);
    VD_REP(alarms);
    VD_REP(predefined_chat);
    // Diver command IDLE by default
    VD_REP(command);
    // Command payload data
    validateCommand(report_in, report_out);
    // Ignore optional data by default
    VD_REP(optional_data);
    if (report_out.optional_data)
    {
      VD_REP(breathing_rate);
      VD_REP(motion_rate);
      VD_REP(pad_space);
    }

    // Additional chat
    // diver.chat.push_back(0);
  }

  void validateReport(const BuddyReport& report_in,
                      const BuddyReport& report_out)
  {
    VD_REP(inited);
    if (!validateInit(report_in, report_out))
      return;

    VD_REP(has_position);
    if (report_out.has_position)
    {
      VD_REPD(north);
      VD_REPD(east);
      VD_REPD(depth);
      VD_REPD(altitude);
    }
    VD_REP(course);
    VD_REPD(speed);

    VD_REP(has_diver);
    if (report_out.has_diver)
    {
      VD_REPD(diver_north);
      VD_REPD(diver_east);
    }

    VD_REP(battery_status);
    VD_REP(leak_info);
    VD_REP(command);

    validateCommand(report_in, report_out);
  }

  void validateReport(const SurfaceReport& report_in,
                      const SurfaceReport& report_out)
  {
    // Master mode - True when pinging, false otherwise
    VD_REP(is_master);
    // Initialization of the frame possible only in master mode
    if (report_out.is_master)
    {
      VD_REP(inited);
      if (!validateInit(report_in, report_out))
        return;
    }

    // Position sent when:
    // a) not in master mode
    // b) when inited in master mode
    if (!report_out.is_master || report_out.inited)
    {
      VD_REPD(north);
      VD_REPD(east);
    }

    // Commands and detailed navigation only when not in master mode.
    if (!report_out.is_master)
    {
      VD_REP(course);
      VD_REPD(speed);
      VD_REP(command);

      validateCommand(report_in, report_out);
    }

    // Diver information only when in master mode and inited
    if (report_out.is_master && report_out.inited)
    {
      VD_REP(has_diver);
      if (report_out.has_diver)
      {
        VD_REPD(diver_north);
        VD_REPD(diver_east);
      }
    }

    // Position sent when:
    // a) not in master mode
    // b) when inited in master mode
    if (!report_out.is_master || report_out.inited)
    {
      VD_REP(predefined_chat);
      // Additional chat.
    }
  }

  DiverReport diver;
  BuddyReport buddy;
  SurfaceReport surface;
};

TEST_F(CaddyCommsStructuresTest, diverReportNoOptional)
{
  DiverReport report_out;
  encodeAndDecodeReport(diver, report_out);
}

TEST_F(CaddyCommsStructuresTest, diverReportOptional)
{
  DiverReport report_out;
  diver.optional_data = true;
  encodeAndDecodeReport(diver, report_out);
}

TEST_F(CaddyCommsStructuresTest, buddyReportPosition)
{
  BuddyReport report_out;
  // Set as Inited and having position
  buddy.inited = true;
  buddy.has_position = true;
  buddy.has_diver = false;
  // Decode and print
  encodeAndDecodeReport(buddy, report_out);
}

TEST_F(CaddyCommsStructuresTest, buddyReportNoPosition)
{
  BuddyReport report_out;
  buddy.inited = true;
  buddy.has_position = false;
  buddy.has_diver = false;
  encodeAndDecodeReport(buddy, report_out);
}

TEST_F(CaddyCommsStructuresTest, buddyReportNoPositionDiver)
{
  BuddyReport report_out;
  buddy.inited = true;
  buddy.has_position = false;
  buddy.has_diver = true;
  encodeAndDecodeReport(buddy, report_out);
}

TEST_F(CaddyCommsStructuresTest, buddyReportPositionDiver)
{
  BuddyReport report_out;
  buddy.inited = true;
  buddy.has_position = true;
  buddy.has_diver = true;
  encodeAndDecodeReport(buddy, report_out);
}

TEST_F(CaddyCommsStructuresTest, buddyReportInit)
{
  BuddyReport report_out;
  buddy.inited = false;
  buddy.has_position = true;
  buddy.has_diver = true;
  encodeAndDecodeReport(buddy, report_out);
}

TEST_F(CaddyCommsStructuresTest, surfaceReportInit)
{
  SurfaceReport report_out;
  surface.is_master = true;
  surface.inited = false;
  surface.has_diver = false;
  encodeAndDecodeReport(surface, report_out);
}

TEST_F(CaddyCommsStructuresTest, surfaceReportNoMaster)
{
  SurfaceReport report_out;
  surface.is_master = false;
  surface.inited = true;
  surface.has_diver = false;
  encodeAndDecodeReport(surface, report_out);
}

TEST_F(CaddyCommsStructuresTest, surfaceReportMasterNoDiver)
{
  SurfaceReport report_out;
  surface.inited = true;
  surface.is_master = true;
  surface.has_diver = true;
  encodeAndDecodeReport(surface, report_out);
}

TEST_F(CaddyCommsStructuresTest, surfaceReportMasterDiver)
{
  SurfaceReport report_out;
  surface.inited = true;
  surface.is_master = true;
  surface.has_diver = true;
  encodeAndDecodeReport(surface, report_out);
}
