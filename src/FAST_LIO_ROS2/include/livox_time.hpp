// RUBI delivery: strict Livox PointCloud2 decoder, MIT license.
#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace legged_livox {
struct Point {float x,y,z,intensity; double offset_ms; uint8_t tag,line;};
inline std::vector<Point> decode(const sensor_msgs::msg::PointCloud2 &msg) {
  using Field=sensor_msgs::msg::PointField;
  const auto field = [&](const char *name, uint8_t type, size_t bytes) {
    for (const auto &f : msg.fields) {
      if (f.name==name && f.datatype==type && f.count==1 &&
          size_t(f.offset)+bytes<=msg.point_step) {return f.offset;}
    }
    throw std::runtime_error(std::string("Livox cloud requires field ")+name);
  };
  const auto x=field("x",Field::FLOAT32,4), y=field("y",Field::FLOAT32,4);
  const auto z=field("z",Field::FLOAT32,4), i=field("intensity",Field::FLOAT32,4);
  const auto t=field("timestamp",Field::FLOAT64,8);
  const auto tag=field("tag",Field::UINT8,1), line=field("line",Field::UINT8,1);
  if(msg.is_bigendian || !msg.point_step ||
      uint64_t(msg.width)*msg.point_step>msg.row_step ||
      uint64_t(msg.row_step)*msg.height>msg.data.size()) {
    throw std::runtime_error("Malformed or big-endian Livox cloud");
  }
  const long double base=static_cast<long double>(msg.header.stamp.sec)*1000000000.L+
    msg.header.stamp.nanosec;
  std::vector<Point> points;
  points.reserve(size_t(msg.width)*msg.height);
  for(uint32_t row=0;row<msg.height;++row) for(uint32_t col=0;col<msg.width;++col) {
    const auto *p=msg.data.data()+size_t(row)*msg.row_step+size_t(col)*msg.point_step;
    Point out{}; double timestamp;
    std::memcpy(&out.x,p+x,4); std::memcpy(&out.y,p+y,4); std::memcpy(&out.z,p+z,4);
    std::memcpy(&out.intensity,p+i,4); std::memcpy(&timestamp,p+t,8);
    out.tag=p[tag];out.line=p[line];
    out.offset_ms=static_cast<double>((static_cast<long double>(timestamp)-base)/1.e6L);
    // FLOAT64 epoch nanoseconds have sub-microsecond rounding; never infer a
    // unit from magnitude, and never invent times from nonrepetitive scan yaw.
    if(!std::isfinite(out.offset_ms) || out.offset_ms < -0.001 || out.offset_ms>500.)
      throw std::runtime_error("Livox timestamp must be absolute ns within 0.5 s of header");
    out.offset_ms=std::max(0.,out.offset_ms);
    if(std::isfinite(out.x)&&std::isfinite(out.y)&&std::isfinite(out.z)) points.push_back(out);
  }
  std::stable_sort(points.begin(),points.end(),[](const Point&a,const Point&b){
    return a.offset_ms<b.offset_ms;});
  return points;
}
}  // namespace legged_livox
