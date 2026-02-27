/*
 * Copyright 2017-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <CLI/CLI.hpp>
#include <mesh_sampling/mesh_sampling.h>

namespace fs = std::filesystem;

using namespace mesh_sampling;

int main(int argc, char ** argv)
{
  CLI::App app("mesh_sampling");
  argv = app.ensure_utf8(argv);

  fs::path in;
  app.add_option("--in", in, "Input mesh (supported by ASSIMP)")->required()->check(CLI::ExistingPath);

  fs::path out;
  app.add_option("--out", out, "Output file (ply, pcd, qc, stl)");

  fs::path convex;
  app.add_option("--convex", convex, "Output convex directory")->check(CLI::ExistingPath);

  bool use_qconvex = false;
  app.add_flag("--use-qconvex", use_qconvex, "Use qconvex executable instead of libqhullcpp");

  unsigned int N;
  app.add_option("--samples", N, "Number of points to sample")->default_val(10000)->check(CLI::PositiveNumber);

  float scale;
  app.add_option("--scale", scale, "Scale factor applied to the mesh")->default_val(1.0)->check(CLI::Range(0, 1));

  std::string cloud_type = "xyz_rgb_normal";
  app.add_option("--type, -t", cloud_type, "Type of cloud to generate (xyz, xyz_rgb, xyz_rgb_normal)");

  bool binary_format;
  app.add_option("--binary, -b", binary_format, "Outputs in binary format (default: false)")->default_val(false);

  bool convert;
  app.add_option("--convert", convert, "Convert from one mesh type to another (supported by ASSIMP)")
      ->default_val(false);

  CLI11_PARSE(app, argc, argv);

  MeshSampling mesh_sampler(in);

  if(!mesh_sampler.check_supported(cloud_type))
  {
    std::cerr << "Type not suppported : ";
    std::copy(mesh_sampling::supported_cloud_type.begin(), mesh_sampling::supported_cloud_type.end(),
              std::ostream_iterator<std::string>(std::cerr, ", "));
    std::cerr << std::endl;
  }

  if(convert)
  {
    mesh_sampler.convertTo(out, binary_format);
  }
  else
  {
    // Only these types are supported for convex generation
    if(cloud_type == "xyz" || cloud_type == "xyz_rgb" || cloud_type == "xyz_normal" || cloud_type == "xyz_rgb_normal")
    {
      auto mesh = mesh_sampler.create_clouds(N, out, ".qc", binary_format);
      if(!convex.empty())
      {
        if(use_qconvex)
          mesh_sampler.create_convexes_qconvex(mesh, convex, true);
        else
          mesh_sampler.create_convexes(mesh, convex, true);
      }
    }
    else
    {
      std::cerr << "Convex generation only supported for cloud types : xyz, xyz_rgb, xyz_normal, xyz_rgb_normal" << std::endl;
    }
  }
  return 0;
}
