#include <mesh_sampling/mesh_sampling_impl.h>

MeshSampling::MeshSampling() : impl_(std::make_unique<Impl>()) {}

MeshSampling::MeshSampling(const fs::path & in_path, float scale) : impl_(std::make_unique<Impl>())
{
  load(in_path, scale);
}

MeshSampling::~MeshSampling() = default;

void MeshSampling::load(const fs::path & in_path, float scale)
{
  impl_->load(in_path, scale);
}

bool MeshSampling::check_supported(const std::string & type, const std::vector<std::string> & supported)
{
  return impl_->check_supported(type, supported);
}

void MeshSampling::convertTo(const fs::path & out_path, bool binary)
{
  impl_->convertTo(out_path, binary);
}

ASSIMPScene * MeshSampling::mesh(const std::string & path)
{
  return impl_->mesh(path);
}

template<typename PointT>
pcl::PointCloud<PointT> MeshSampling::cloud(unsigned N)
{
  return impl_->cloud<PointT>(N);
}

template<typename PointT>
pcl::PointCloud<PointT> MeshSampling::create_cloud(const aiScene * scene,
                                                   unsigned N,
                                                   const fs::path & out_path,
                                                   bool binary_mode)
{
  return impl_->create_cloud<PointT>(scene, N, out_path, binary_mode);
}

template<typename PointT>
std::map<std::string, pcl::PointCloud<PointT>> MeshSampling::create_clouds(unsigned N,
                                                                           const fs::path & out_path,
                                                                           const std::string & extension,
                                                                           bool binary_mode)
{
  return impl_->create_clouds<PointT>(N, out_path, extension, binary_mode);
}

template<typename PointT>
std::string MeshSampling::create_convex(const pcl::PointCloud<PointT> & cloud, const fs::path & out_path)
{
  return impl_->create_convex<PointT>(cloud, out_path);
}

template<typename PointT>
std::map<std::string, std::string> MeshSampling::create_convexes(
    const std::map<std::string, pcl::PointCloud<PointT>> & clouds,
    const fs::path & out_path,
    bool stop_on_fail)
{
  return impl_->create_convexes<PointT>(clouds, out_path, stop_on_fail);
}

// Explicit instantiations (only ones you need)
// template class MeshSampling::Impl;
template pcl::PointCloud<pcl::PointXYZ> MeshSampling::cloud(unsigned);
template pcl::PointCloud<pcl::PointXYZ> MeshSampling::create_cloud(const aiScene *, unsigned, const fs::path &, bool);
template std::map<std::string, pcl::PointCloud<pcl::PointXYZ>> MeshSampling::create_clouds(unsigned,
                                                                                           const fs::path &,
                                                                                           const std::string &,
                                                                                           bool);
template std::string MeshSampling::create_convex(const pcl::PointCloud<pcl::PointXYZ> &, const fs::path &);
template std::map<std::string, std::string> MeshSampling::create_convexes(
    const std::map<std::string, pcl::PointCloud<pcl::PointXYZ>> &,
    const fs::path &,
    bool);

template pcl::PointCloud<pcl::PointXYZRGB> MeshSampling::cloud(unsigned);
template pcl::PointCloud<pcl::PointXYZRGB> MeshSampling::create_cloud(const aiScene *,
                                                                      unsigned,
                                                                      const fs::path &,
                                                                      bool);
template std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> MeshSampling::create_clouds(unsigned,
                                                                                              const fs::path &,
                                                                                              const std::string &,
                                                                                              bool);
template std::string MeshSampling::create_convex(const pcl::PointCloud<pcl::PointXYZRGB> &, const fs::path &);
template std::map<std::string, std::string> MeshSampling::create_convexes(
    const std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> &,
    const fs::path &,
    bool);

template pcl::PointCloud<pcl::PointNormal> MeshSampling::cloud(unsigned);
template pcl::PointCloud<pcl::PointNormal> MeshSampling::create_cloud(const aiScene *,
                                                                      unsigned,
                                                                      const fs::path &,
                                                                      bool);
template std::map<std::string, pcl::PointCloud<pcl::PointNormal>> MeshSampling::create_clouds(unsigned,
                                                                                              const fs::path &,
                                                                                              const std::string &,
                                                                                              bool);
template std::string MeshSampling::create_convex(const pcl::PointCloud<pcl::PointNormal> &, const fs::path &);
template std::map<std::string, std::string> MeshSampling::create_convexes(
    const std::map<std::string, pcl::PointCloud<pcl::PointNormal>> &,
    const fs::path &,
    bool);

template pcl::PointCloud<pcl::PointXYZRGBNormal> MeshSampling::cloud(unsigned);
template pcl::PointCloud<pcl::PointXYZRGBNormal> MeshSampling::create_cloud(const aiScene *,
                                                                            unsigned,
                                                                            const fs::path &,
                                                                            bool);
template std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>> MeshSampling::create_clouds(unsigned,
                                                                                                    const fs::path &,
                                                                                                    const std::string &,
                                                                                                    bool);
template std::string MeshSampling::create_convex(const pcl::PointCloud<pcl::PointXYZRGBNormal> &, const fs::path &);
template std::map<std::string, std::string> MeshSampling::create_convexes(
    const std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>> &,
    const fs::path &,
    bool);
