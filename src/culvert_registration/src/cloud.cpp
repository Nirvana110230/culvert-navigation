#include "registration.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <openssl/evp.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace culvert {
Json::Value number(double value) { return std::isfinite(value) ? Json::Value(value) : Json::Value(); }

bool valid_pose(const Pose& p, double eps) {
  if (!p.allFinite() || (p.row(3)-Eigen::RowVector4d(0,0,0,1)).norm() > eps) return false;
  const Eigen::Matrix3d r = p.topLeftCorner<3,3>();
  return (r.transpose()*r - Eigen::Matrix3d::Identity()).norm() <= eps &&
         std::abs(r.determinant()-1.0) <= eps;
}

std::string sha256(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("map_not_readable");
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("sha256_init_failed");
  std::array<char, 65536> buffer;
  while (file) {
    file.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(ctx.get(), buffer.data(), file.gcount()) != 1)
      throw std::runtime_error("sha256_update_failed");
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (!file.eof() || !EVP_DigestFinal_ex(ctx.get(), digest, &length) || length != 32)
    throw std::runtime_error("sha256_read_failed");
  std::ostringstream out;
  for (unsigned i = 0; i < length; ++i) out << std::hex << std::setfill('0') << std::setw(2) << int(digest[i]);
  return out.str();
}

Cloud::Ptr read_pcd(const std::string& path, int limit, size_t max_bytes) {
  if (!std::filesystem::is_regular_file(path) || std::filesystem::file_size(path) > max_bytes)
    throw std::runtime_error("pcd_file_missing_or_too_large");
  // Check dimensions before PCL allocates the binary buffer.
  std::ifstream stream(path, std::ios::binary);
  std::string line;
  size_t bytes = 0, points = 0, width = 0, height = 1;
  bool data = false;
  while (std::getline(stream,line) && (bytes += line.size()+1) <= 65536) {
    std::istringstream words(line);
    std::string key;
    words >> key;
    if (key == "POINTS") words >> points;
    if (key == "WIDTH") words >> width;
    if (key == "HEIGHT") words >> height;
    if (key == "DATA") { data = true; break; }
  }
  if (!data || !width || !height || width > size_t(limit)/height ||
      points != width*height || points > size_t(limit)) throw std::runtime_error("pcd_invalid_dimensions");
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path, blob) < 0) throw std::runtime_error("pcd_read_failed");
  for (const auto* name : {"x","y","z"}) {
    auto field = std::find_if(blob.fields.begin(), blob.fields.end(),
                              [name](const auto& f) { return f.name == name; });
    if (field == blob.fields.end() || field->datatype != pcl::PCLPointField::FLOAT32 || field->count != 1)
      throw std::runtime_error("pcd_xyz_float32_required");
  }
  Cloud::Ptr cloud(new Cloud);
  pcl::fromPCLPointCloud2(blob,*cloud);
  if (cloud->size()!=points) throw std::runtime_error("pcd_size_mismatch");
  for (const auto& p : *cloud)
    if (!p.getVector3fMap().allFinite()) throw std::runtime_error("nonfinite_points");
  return cloud;
}
Cloud::Ptr voxel(const Cloud::Ptr& points, float leaf) {
  Cloud::Ptr result(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(leaf,leaf,leaf);
  filter.setInputCloud(points);
  filter.filter(*result);
  return result;
}
Tree::Ptr make_tree(const Cloud::Ptr& points) {
  Tree::Ptr tree(new Tree);
  tree->setInputCloud(points);
  return tree;
}
static Cloud::Ptr crop(const Cloud::Ptr& full, Tree& tree, const pcl::PointXYZ& center, double radius) {
  std::vector<int> ids;
  std::vector<float> distances;
  tree.radiusSearch(center, radius, ids, distances, max_local_points+1);
  if (ids.size() > max_local_points) throw std::runtime_error("submap_point_limit");
  if (ids.size() < min_points) throw std::runtime_error("insufficient_submap");
  std::sort(ids.begin(),ids.end());
  Cloud::Ptr result(new Cloud);
  result->reserve(ids.size());
  for (int i : ids) result->push_back((*full)[i]);
  return result;
}
Target& local_target(Map& map, const Eigen::Vector3d& center, double radius, bool& hit) {
  const double grid_m=std::min(1.0,radius/4.0);
  const Eigen::Vector3d anchor=(center/grid_m).array().round().matrix()*grid_m;
  const double effective_radius=radius+std::sqrt(3.0)*grid_m/2.0;
  for (auto it=map.cache.begin(); it!=map.cache.end(); ++it) {
    if (it->center == anchor && it->radius == radius) {
      map.cache.splice(map.cache.begin(),map.cache,it);
      hit=true;
      return map.cache.front();
    }
  }
  hit=false;
  const pcl::PointXYZ point(anchor.x(),anchor.y(),anchor.z());
  Target next;
  next.center=anchor; next.radius=radius; next.grid_m=grid_m; next.effective_radius=effective_radius;
  next.rough=crop(map.rough,*map.rough_tree,point,effective_radius);
  next.fine=crop(map.fine,*map.fine_tree,point,effective_radius);
  next.min_corner=next.max_corner=next.fine->front().getVector3fMap().cast<double>();
  for (const auto& p : *next.fine) {
    const Eigen::Vector3d q=p.getVector3fMap().cast<double>();
    next.min_corner=next.min_corner.cwiseMin(q);
    next.max_corner=next.max_corner.cwiseMax(q);
  }
  next.rough_tree=make_tree(next.rough);
  next.fine_tree=make_tree(next.fine);
  next.normals.resize(next.fine->size());
  next.normal_status.resize(next.fine->size());
  if (map.cache.size() == 2) map.cache.pop_back();
  map.cache.emplace_front(std::move(next));
  return map.cache.front();
}
}  // namespace culvert
