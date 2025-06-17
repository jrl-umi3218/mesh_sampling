#include "mesh_sampling/mesh_sampling.h"

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

CloudT MeshSampling::cloud(unsigned N)
{
  return impl_->cloud(N);
}

CloudT MeshSampling::create_cloud(const aiScene * scene, unsigned N, const fs::path & out_path, bool binary_mode)
{
  return impl_->create_cloud(scene, N, out_path, binary_mode);
}

std::map<std::string, CloudT> MeshSampling::create_clouds(unsigned N,
                                                          const fs::path & out_path,
                                                          const std::string & extension,
                                                          bool binary_mode)
{
  return impl_->create_clouds(N, out_path, extension, binary_mode);
}

std::string MeshSampling::create_convex(const CloudT & cloud, const fs::path & out_path)
{
  return impl_->create_convex(cloud, out_path);
}

std::map<std::string, std::string> MeshSampling::create_convexes(const std::map<std::string, CloudT> & clouds,
                                                                 const fs::path & out_path,
                                                                 bool stop_on_fail)
{
  return impl_->create_convexes(clouds, out_path, stop_on_fail);
}
