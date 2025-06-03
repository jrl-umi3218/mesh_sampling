/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/algorithm/string.hpp>
#include <filesystem>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <mesh_sampling/mesh_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
namespace fs = std::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace mesh_sampling;

int main(int argc, char ** argv)
{
  po::variables_map vm;
  po::options_description desc("Allowed Options");

  // clang-format off
  desc.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input mesh (supported by ASSIMP)")
    ("out", po::value<std::string>(), "Output file (ply, pcd, qc, stl)")
    ("samples", po::value<unsigned>()->default_value(10000), "Number of points to sample")
    ("scale", po::value<float>()->default_value(1.0), "Scale factor applied to the mesh")
    ("type", po::value<std::string>()->default_value("xyz_rgb_normal"), "Type of cloud to generate (xyz, xyz_rgb, xyz_rgb_normal)")
    ("binary", po::bool_switch()->default_value(false), "Outputs in binary format (default: false)")
    ("convert", po::bool_switch()->default_value(false), "Convert from one mesh type to another (supported by ASSIMP)");
  // clang-format on

  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);

  // parse arguments and save them in the variable map (vm)
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);

  if(!vm.count("in") || !vm.count("out") || vm.count("help"))
  {
    std::cout << "mesh_sampling is a command-line tool to sample pointcloud from CAD meshes\n\n";
    std::cout << "Usage: mesh_sampling [options] in out\n\n";
    std::cout << desc << "\n";
    return !vm.count("help");
  }
  bool convert = vm["convert"].as<bool>();

  std::string in_path = vm["in"].as<std::string>();
  std::string out_path = vm["out"].as<std::string>();
  fs::path in_p(in_path);
  fs::path out_p(out_path);
  const auto out_extension = boost::algorithm::to_lower_copy(out_p.extension().string());
  const unsigned N = vm["samples"].as<unsigned>();
  const std::string cloud_type = vm["type"].as<std::string>();
  const bool binary_format = vm["binary"].as<bool>();

  MeshSampling mesh_sampler(in_p);

  mesh_sampler.check_supported(supported_cloud_type, cloud_type);

  if(convert)
  {
    mesh_sampler.convertTo(out_p, binary_format);
  }
  else
  {
    if(cloud_type == "xyz")
    {
      auto mesh = mesh_sampler.create_cloud<pcl::PointXYZ>(mesh_sampler.mesh(in_p)->scene(), N, out_p, binary_format);
      mesh_sampler.create_convex(mesh, "/tmp/test-ch.txt");
    }
    else if(cloud_type == "xyz_rgb")
    {
      mesh_sampler.create_cloud<pcl::PointXYZRGB>(mesh_sampler.mesh(in_p)->scene(), N, out_p, binary_format);
    }
    else if(cloud_type == "xyz_normal")
    {
      mesh_sampler.create_cloud<pcl::PointNormal>(mesh_sampler.mesh(in_p)->scene(), N, out_p, binary_format);
    }
    else if(cloud_type == "xyz_rgb_normal")
    {
      mesh_sampler.create_cloud<pcl::PointXYZRGBNormal>(mesh_sampler.mesh(in_p)->scene(), N, out_p, binary_format);
    }
  }

  return 0;
}