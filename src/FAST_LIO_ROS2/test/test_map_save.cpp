#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <unistd.h>
#include "map_save.hpp"
TEST(MapSave, EmptyFailsAndValidMapRoundTripsWithoutAccidentalOverwrite) {
  auto dir=std::filesystem::temp_directory_path()/("fastlio_save_"+std::to_string(getpid()));
  auto file=(dir/"nested/map.pcd").string();pcl::PointCloud<pcl::PointXYZ> cloud;
  EXPECT_THROW(fast_lio::saveMap(file,cloud,false),std::runtime_error);
  cloud.push_back(pcl::PointXYZ(1,2,3));fast_lio::saveMap(file,cloud,false);
  EXPECT_THROW(fast_lio::saveMap(file,cloud,false),std::runtime_error);
  pcl::PointCloud<pcl::PointXYZ> restored;ASSERT_EQ(pcl::io::loadPCDFile(file,restored),0);
  ASSERT_EQ(restored.size(),1U);EXPECT_FLOAT_EQ(restored[0].z,3);
  cloud.push_back(pcl::PointXYZ(4,5,6));fast_lio::saveMap(file,cloud,true);
  ASSERT_EQ(pcl::io::loadPCDFile(file,restored),0);EXPECT_EQ(restored.size(),2U);
  std::filesystem::remove_all(dir);
}
