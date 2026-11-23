// Apache-2.0. Reuse the accumulator's validated RGHM reader for frozen navigation.
#include <fstream>
#include <iomanip>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rubi_global_heightmap_wrapper/global_grid.hpp"
class FrozenHeightmap : public rclcpp::Node {
 public:
  FrozenHeightmap() : Node("frozen_heightmap") {
    const auto file=declare_parameter<std::string>("file","");
    const auto frame=declare_parameter<std::string>("frame","map");
    const auto topic=declare_parameter<std::string>("topic","/legged/height");
    const auto resolution=declare_parameter<double>("expected_resolution",.05);
    std::ifstream stream(file);
    if(!stream) throw std::runtime_error("Cannot open height map: "+file);
    // GlobalGrid::load intentionally requires matching live-map configuration.
    // Read only the header to construct that geometry, then let its strict
    // reader validate the complete file (including every cell and trailing data).
    std::string magic; int version=0; rubi::Geometry geometry;
    float max_variance=0.; std::int64_t stamp=0; std::size_t count=0;
    if(!(stream >> magic >> version >> std::quoted(geometry.frame) >> geometry.resolution
         >> geometry.origin_x >> geometry.origin_y >> geometry.cols >> geometry.rows
         >> max_variance >> stamp >> count) || magic!="RUBI_GLOBAL_HEIGHTMAP" || version!=1)
      throw std::runtime_error("Malformed frozen height map header");
    rubi::GlobalGrid grid(geometry,max_variance);
    stream.clear(); stream.seekg(0); grid.load(stream);
    if(grid.geometry().frame!=frame) throw std::runtime_error("Height map frame does not match navigation frame");
    if(std::abs(grid.geometry().resolution-resolution)>1e-8)
      throw std::runtime_error("Height map resolution does not match planner map_resolution_m");
    const auto surface=grid.surface();
    if(surface.empty()) throw std::runtime_error("Height map has no usable surface cells");
    sensor_msgs::msg::PointCloud2 cloud; cloud.header.frame_id=frame; cloud.header.stamp=now();
    sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(surface.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
    for(const auto &p:surface) { *x=p.x; *y=p.y; *z=p.z; ++x; ++y; ++z; }
    cloud.is_dense=true;
    pub_=create_publisher<sensor_msgs::msg::PointCloud2>(topic,rclcpp::QoS(1).transient_local());
    pub_->publish(cloud);
    RCLCPP_INFO(get_logger(),"Frozen height published: %s (%zu cells, frame=%s, resolution=%.3f m)",
      file.c_str(),surface.size(),frame.c_str(),resolution);
  }
 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<FrozenHeightmap>()); }
  catch(const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("frozen_heightmap"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
