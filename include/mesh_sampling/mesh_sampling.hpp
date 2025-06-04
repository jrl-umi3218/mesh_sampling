
#pragma once
#include "mesh_sampling.h"
#include <filesystem>
#include <libqhullcpp/QhullPoints.h>
#include <map>
#include <string>

namespace mesh_sampling {
template<typename PointT>
void MeshSampling::create_convex(const pcl::PointCloud<PointT> & cloud, const fs::path & out_path)
{
  if(cloud.empty())
  {
    cerr << "Error: Empty point cloud." << endl;
    return;
  }

  std::ofstream out(out_path);
  if(!out.is_open())
  {
    std::cerr << "Error: Could not open file: " << out_path << std::endl;
    return;
  }

  // Convert PCL cloud to a flat array for Qhull input
  std::vector<double> qhull_input;
  for(const auto & pt : cloud)
  {
    qhull_input.push_back(pt.x);
    qhull_input.push_back(pt.y);
    qhull_input.push_back(pt.z);
  }

  // Create Qhull object
  Qhull qhull;
  try
  {
    qhull.runQhull("pcl_input", 3, cloud.size(), qhull_input.data(), "Qt"); // 3D, triangulate option
  }
  catch(const std::exception & e)
  {
    cerr << "Qhull exception: " << e.what() << endl;
    return;
  }

  int dim = qhull.hullDimension();
  int numfacets = qhull.facetList().count();
  int totneighbors = numfacets * dim; // not accurate for non-simplicial facets
  out << dim << "\n" << qhull.points().size() << " " << numfacets << " " << totneighbors / 2 << "\n";

  std::vector<std::vector<double>> points;
  for(QhullPoints::ConstIterator i = qhull.points().begin(); i != qhull.points().end(); ++i)
  {
    QhullPoint point = *i;
    points.push_back(point.toStdVector());
  }

  for(const auto & point : points)
  {
    for(size_t i = 0; i < point.size(); ++i)
    {
      out << std::setw(6) << point[i] << (i < point.size() - 1 ? " " : "\n");
    }
  }

  QhullFacetList facets = qhull.facetList();
  std::vector<std::vector<int>> facetVertices;

  QhullFacetListIterator j(facets);
  while(j.hasNext())
  {
    QhullFacet f = j.next();
    std::vector<int> vertices;
    if(!f.isGood())
    {
      continue;
    }
    else if(!f.isTopOrient() && f.isSimplicial())
    {
      QhullVertexSet vs = f.vertices();
      vertices.push_back(vs[1].point().id());
      vertices.push_back(vs[0].point().id());
      for(int i = 2; i < static_cast<int>(vs.size()); ++i)
      {
        vertices.push_back(vs[i].point().id());
      }
    }
    else
    {
      QhullVertexSetIterator k(f.vertices());
      while(k.hasNext())
      {
        QhullVertex vertex = k.next();
        vertices.push_back(vertex.point().id());
      }
    }
    facetVertices.push_back(vertices);
  }

  for(const auto & vertices : facetVertices)
  {
    out << vertices.size() << " ";
    for(int v : vertices)
    {
      out << v << " ";
    }
    out << "\n";
  }

  out.close();
  std::cout << "Convex file saved to " << out_path << std::endl;
}

template<typename PointT>
void MeshSampling::create_convexes(const std::map<std::string, pcl::PointCloud<PointT>> &clouds, const fs::path &out_path){
  if(!fs::is_directory(out_path)){
    std::cerr << "[Error] create_convexes, out_path has to be a directory" << std::endl;
    return;
  }
  for(const auto & cloud : clouds){
    create_convex(cloud.second, out_path / (fs::path(cloud.first).filename().stem().string() + "-ch.txt"));
  }
}

template<typename PointT>
std::map<std::string, pcl::PointCloud<PointT>> MeshSampling::create_clouds(unsigned N, const fs::path & out_path, const std::string & extension, bool binary_mode)
{
  std::map<std::string, pcl::PointCloud<PointT>> clouds;
  if(!out_path.empty()){
    if(!fs::is_directory(out_path) && meshes_.size() > 1)
    {
      std::cerr << "[Error] create_clouds, out_path is not a directory" << std::endl;;
      return {};
    }

    if(extension.empty()){
      std::cerr << "[Error] create_clouds, extension is not set" << std::endl;
      return {};
    }

    for(const auto & mesh : meshes_)
    {
      auto path = fs::is_directory(out_path) ? out_path / (fs::path(mesh.first).filename().stem().string() + extension) : out_path;
      auto cloud = create_cloud<PointT>(mesh.second->scene(), N, path, binary_mode);
      clouds.insert({mesh.first, cloud});
    }
  }
  else{
    for(const auto & mesh : meshes_)
    {
      auto cloud = create_cloud<PointT>(mesh.second->scene(), N, {}, binary_mode);
      clouds.insert({mesh.first, cloud});
    }
  }

  return clouds;
}

template<typename PointT>
pcl::PointCloud<PointT> MeshSampling::cloud(const unsigned N){
  WeightedRandomSampling<PointT> sampler(meshes_.begin()->second->scene());
  auto cloud = sampler.weighted_random_sampling(N);
  return *cloud;
}

template<typename PointT>
pcl::PointCloud<PointT> MeshSampling::create_cloud(const aiScene * scene,
                                                   const unsigned N,
                                                   const fs::path & out_path,
                                                   bool binary_mode)
{
  WeightedRandomSampling<PointT> sampler(scene);
  auto cloud = sampler.weighted_random_sampling(N);

  if(!out_path.empty() && !fs::is_directory(out_path))
  {
    auto extension = boost::algorithm::to_lower_copy(out_path.extension().string());
    bool success = true;
    if(extension == ".pcd")
    {
      success = pcl::io::savePCDFile(out_path, *cloud, binary_mode) == 0;
    }
    else if(extension == ".ply")
    {
      success = pcl::io::savePLYFile(out_path, *cloud, binary_mode) == 0;
    }
    else if(extension == ".qc")
    {
      success = mesh_sampling::io::saveQhullFile(out_path, *cloud);
    }
    else if(extension == ".stl")
    {
      pcl::ConvexHull<PointT> convex{};
      const auto shared_cloud = typename pcl::PointCloud<PointT>::ConstPtr(cloud.release());
      convex.setInputCloud(shared_cloud);
      pcl::PolygonMesh mesh;
      convex.reconstruct(mesh);
      pcl::PointCloud<PointT> cloud{};
      pcl::fromPCLPointCloud2(mesh.cloud, cloud);

      // Compute the convex center
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      for(auto & poly : mesh.polygons)
      {
        if(poly.vertices.size() != 3)
        {
          throw std::runtime_error("pcl::ConvexHull did not reconstruct a triangular mesh");
        }
        center += (cloud[poly.vertices[0]].getVector3fMap() + cloud[poly.vertices[1]].getVector3fMap()
                   + cloud[poly.vertices[2]].getVector3fMap())
                  / (3 * mesh.polygons.size());
      }

      // Make sure all normals point away from the mesh center
      // Since the mesh is convex this is enough to ensure the normals are consistent
      for(auto & poly : mesh.polygons)
      {
        Eigen::Vector3f p1 = cloud[poly.vertices[0]].getVector3fMap();
        Eigen::Vector3f p2 = cloud[poly.vertices[1]].getVector3fMap();
        Eigen::Vector3f p3 = cloud[poly.vertices[2]].getVector3fMap();
        Eigen::Vector3f n = (p2 - p1).cross(p3 - p1);
        if(n.dot(center - p1) > 0)
        {
          std::swap(poly.vertices[1], poly.vertices[2]);
        }
      }
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
      success = pcl::io::savePolygonFileSTL(out_path, mesh, binary_mode);
#else
      success = pcl::io::savePolygonFileSTL(out, mesh);
#endif
    }
    else
    {
      std::cerr << "Output pointcloud type " << extension << " is not supported";
      return pcl::PointCloud<PointT>();
    }

    if(!success)
    {
      std::cerr << "Saving to " << out_path << " failed." << std::endl;
      return pcl::PointCloud<PointT>();
    }
    else
    {
      std::cout << "Saving cloud to " << out_path << " success" << std::endl;
    }
  }

  return *cloud;
}
}