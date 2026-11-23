#pragma once
#include <filesystem>
#include <stdexcept>
#include <pcl/io/pcd_io.h>
namespace fast_lio {
template<class Cloud> void saveMap(const std::string &file, const Cloud &cloud, bool overwrite) {
  namespace fs=std::filesystem;
  if(file.empty()) throw std::runtime_error("pcd_map/map_file_path is empty");
  if(cloud.empty()) throw std::runtime_error("No accumulated map points yet (publish.map_en must be true)");
  fs::path path(file);
  if(fs::exists(path) && !overwrite) throw std::runtime_error("PCD exists; choose a new pcd_map or set pcd_overwrite:=true: "+file);
  if(!path.parent_path().empty()) fs::create_directories(path.parent_path());
  const auto temporary=path.string()+".tmp";
  try {
    if(pcl::PCDWriter{}.writeBinary(temporary,cloud)!=0) throw std::runtime_error("PCD writer failed: "+file);
    fs::rename(temporary,path);
  } catch(...) { std::error_code ec;fs::remove(temporary,ec);throw; }
}
}
