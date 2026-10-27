// MIT License. Keep scan timestamps and point fields; never use a latest dynamic pose.
#include <cmath>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include "legged_navigation/sector_filter.hpp"

class ObstacleCloud : public rclcpp::Node {
  double minimum_, maximum_, max_age_, min_body_height_, max_body_height_;
  std::string base_, last_state_;
  std::unique_ptr<tf2_ros::Buffer> mounts_;
  std::unique_ptr<legged_navigation::SectorFilter> filter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_, collision_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_;
  void state(const std::string & value) {
    std_msgs::msg::String msg; msg.data=value; status_->publish(msg);
    if(value!=last_state_) {RCLCPP_INFO(get_logger(), "Collision input: %s",value.c_str()); last_state_=value;}
  }
 public:
  ObstacleCloud():Node("obstacle_cloud") {
    minimum_=declare_parameter("min_range",.6);maximum_=declare_parameter("max_range",5.);
    const auto front_minimum=declare_parameter("front_min_range",minimum_);
    const auto front_angle=declare_parameter("front_half_angle_deg",0.0);
    filter_=std::make_unique<legged_navigation::SectorFilter>(
      minimum_,maximum_,front_minimum,front_angle);
    base_=declare_parameter<std::string>("base_frame","base_link");
    min_body_height_=declare_parameter("min_body_height",-.35);
    max_body_height_=declare_parameter("max_body_height",1.5);
    max_age_=declare_parameter("max_collision_age",.4);
    if(!std::isfinite(minimum_) || !std::isfinite(maximum_) || !std::isfinite(max_age_) ||
       !std::isfinite(min_body_height_) || !std::isfinite(max_body_height_) ||
       !(minimum_>=0 && maximum_>minimum_ && max_age_>0 && min_body_height_<max_body_height_) || base_.empty())
      throw std::invalid_argument("Invalid range/frame/age settings");
    RCLCPP_INFO(get_logger(),
      "Range filter: default %.2f m, front %.2f m within +/-%.1f deg of %s +X; max %.2f m",
      minimum_,front_minimum,front_angle,base_.c_str(),maximum_);
    RCLCPP_INFO(get_logger(),"Obstacle height band: %.2f .. %.2f m in %s (not map Z)",
      min_body_height_,max_body_height_,base_.c_str());
    mounts_=std::make_unique<tf2_ros::Buffer>(get_clock());
    status_=create_publisher<std_msgs::msg::String>("/legged/collision_input_state",rclcpp::QoS(1).transient_local());
    pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/legged/obstacles",rclcpp::SensorDataQoS());
    collision_=create_publisher<sensor_msgs::msg::PointCloud2>("/legged/collision_points",rclcpp::SensorDataQoS());
    // Fixed mounting chain only. Dynamic localization is deliberately not subscribed.
    static_=create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for(const auto &t:msg->transforms) mounts_->setTransform(t,"fixed_mount",true);
      });
    sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        int offsets[3]={-1,-1,-1};
        for(const auto &f:msg->fields) for(int i=0;i<3;++i)
          if(f.name==std::string(1,"xyz"[i]) && f.datatype==7 && f.count==1 && f.offset+4<=msg->point_step) offsets[i]=f.offset;
        if(offsets[0]<0||offsets[1]<0||offsets[2]<0||msg->is_bigendian||!msg->point_step||
           uint64_t(msg->width)*msg->point_step>msg->row_step||
           uint64_t(msg->row_step)*msg->height>msg->data.size()||msg->header.frame_id.empty()) {
          state("invalid_cloud"); return;
        }
        tf2::Transform mount;mount.setIdentity();
        if(base_!=msg->header.frame_id) {
          try {
            const auto t=mounts_->lookupTransform(base_,msg->header.frame_id,tf2::TimePointZero);
            const auto &q=t.transform.rotation;const auto &v=t.transform.translation;
            mount=tf2::Transform(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(v.x,v.y,v.z));
          } catch(const tf2::TransformException &) {state("missing_static_mount");return;}
        }
        const auto &r=mount.getBasis();
        const std::array<double,6> rotation_xy={r[0][0],r[0][1],r[0][2],r[1][0],r[1][1],r[1][2]};
        sensor_msgs::msg::PointCloud2 out;
        out.header=msg->header;out.fields=msg->fields;out.point_step=msg->point_step;
        out.height=1;out.is_dense=true;out.data.reserve(msg->data.size());
        size_t finite=0;
        for(uint32_t row=0;row<msg->height;++row) for(uint32_t col=0;col<msg->width;++col) {
          const auto *p=msg->data.data()+size_t(row)*msg->row_step+size_t(col)*msg->point_step;
          float v[3];for(int i=0;i<3;++i) std::memcpy(&v[i],p+offsets[i],4);
          const double d=double(v[0])*v[0]+double(v[1])*v[1]+double(v[2])*v[2];
          if(!std::isfinite(d)) continue;
          ++finite;
          if(!filter_->keep(v[0],v[1],v[2],rotation_xy)) continue;
          // Select in robot coordinates, but preserve sensor coordinates in the
          // published cloud so Nav2 keeps the correct ray-clearing origin.
          const double body_z=r[2][0]*v[0]+r[2][1]*v[1]+r[2][2]*v[2]+mount.getOrigin().z();
          if(body_z>=min_body_height_ && body_z<=max_body_height_)
            out.data.insert(out.data.end(),p,p+msg->point_step);
        }
        if(!finite) {state("no_finite_returns");return;}
        out.width=out.data.size()/out.point_step;out.row_step=out.width*out.point_step;
        // Costmap retains sensor frame/origin for ray clearing.
        pub_->publish(out);
        const double age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
        if(age<-.1||age>max_age_) {state("stale_collision_cloud");return;}
        out.header.frame_id=base_;
        for(uint32_t col=0;col<out.width;++col) {
          auto *p=out.data.data()+size_t(col)*out.point_step;float v[3];
          for(int i=0;i<3;++i) std::memcpy(&v[i],p+offsets[i],4);
          const auto v2=mount*tf2::Vector3(v[0],v[1],v[2]);
          for(int i=0;i<3;++i) {v[i]=float(v2[i]);std::memcpy(p+offsets[i],&v[i],4);}
        }
        collision_->publish(out);state("ready");
      });
    state("waiting_for_scan_and_mount");
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ObstacleCloud>());rclcpp::shutdown();
}
