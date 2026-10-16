#include <gtest/gtest.h>
#include "livox_time.hpp"
#include "imu_rotation.hpp"

sensor_msgs::msg::PointCloud2 cloud() {
  using F=sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 c;c.header.stamp.sec=1791960000;c.header.stamp.nanosec=123456789;
  c.width=2;c.height=2;c.point_step=26;c.row_step=56;c.data.resize(112);
  const char *names[]={"x","y","z","intensity","tag","line","timestamp"};
  const int offsets[]={0,4,8,12,16,17,18};
  for(int i=0;i<7;++i) {F f;f.name=names[i];f.offset=offsets[i];f.count=1;
    f.datatype=i<4?F::FLOAT32:i<6?F::UINT8:F::FLOAT64;c.fields.push_back(f);}
  for(int i=0;i<4;++i) {
    auto *p=c.data.data()+i/2*c.row_step+i%2*c.point_step;
    float x=1.f,y=.2f,z=.3f,intensity=4.f;double t=1791960000123456789.L+i*25000000.L;
    std::memcpy(p,&x,4);std::memcpy(p+4,&y,4);std::memcpy(p+8,&z,4);
    std::memcpy(p+12,&intensity,4);p[16]=0;p[17]=i;std::memcpy(p+18,&t,8);
  }return c;
}
TEST(LivoxTime, EpochNanosecondsMatchCustomOffsetsAndRespectRowPadding) {
  const auto p=legged_livox::decode(cloud());ASSERT_EQ(p.size(),4U);
  for(size_t i=0;i<p.size();++i) {
    const uint32_t custom_offset_ns=i*25000000;
    EXPECT_NEAR(p[i].offset_ms,custom_offset_ns/1e6,0.001);
    EXPECT_FLOAT_EQ(p[i].x,1.f);EXPECT_EQ(p[i].line,i);
  }
}
TEST(LivoxTime, MissingOrWrongUnitTimesAreRejected) {
  auto c=cloud();c.fields.pop_back();EXPECT_THROW(legged_livox::decode(c),std::runtime_error);
  c=cloud();double seconds=1791960000.123;std::memcpy(c.data.data()+18,&seconds,8);
  EXPECT_THROW(legged_livox::decode(c),std::runtime_error);
  c=cloud();c.data.resize(10);EXPECT_THROW(legged_livox::decode(c),std::runtime_error);
}
TEST(LivoxRotation, UpsideDownRotatesBothVectorsWithoutChangingMagnitude) {
  struct Imu {float gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z;} imu{1,2,3,0,0,-1};
  livox_ros::rotate_imu(180,0,0,imu);
  EXPECT_FLOAT_EQ(imu.gyro_x,1);EXPECT_NEAR(imu.gyro_y,-2,1e-6);
  EXPECT_NEAR(imu.gyro_z,-3,1e-6);EXPECT_NEAR(imu.acc_z,1,1e-6);
}
