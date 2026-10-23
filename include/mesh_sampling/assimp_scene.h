#pragma once

#include <Eigen/Core>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <stdexcept>
namespace fs = std::filesystem;

namespace mesh_sampling
{

struct ASSIMPScene
{
  ASSIMPScene(const std::string & model_path) : modelPath_(model_path)
  {
    loadScene();
  }

  ASSIMPScene(const std::string & model_path, const Eigen::Vector3f & scale) : modelPath_(model_path)
  {
    aiMatrix4x4 transform;
    transform.a1 = scale.x();
    transform.b2 = scale.y();
    transform.c3 = scale.z();

    importer.SetPropertyBool(AI_CONFIG_PP_PTV_ADD_ROOT_TRANSFORMATION, true);
    importer.SetPropertyMatrix(AI_CONFIG_PP_PTV_ROOT_TRANSFORMATION, transform);
    importer.SetPropertyBool(AI_CONFIG_PP_PTV_KEEP_HIERARCHY, true);

    loadScene(aiProcess_PreTransformVertices);
    correctNormalsForAssimp53x(scale);
  }

  /**
   * @brief Do not store the returned pointer, only use it
   *
   * @return
   */
  const aiScene * scene() const
  {
    return scene_;
  }

  /**
   * @brief Export the scene to a file in a format supported by ASSIMP
   *
   * @param path Export path
   */
  void exportScene(const std::string & path, const bool binary = true)
  {
    fs::path out_path(path);
    auto ext = out_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    if(ext.empty())
    {
      throw std::runtime_error("Could't export scene " + modelPath_ + " to " + path + ": invalid extension");
    }
    ext.erase(0, 1); // remove leading "."
    if(binary) ext += "b"; // ASSIMP suffixes binary export formats with "b"
    Assimp::Exporter exporter;
    aiReturn ret = exporter.Export(scene_, ext, path);
    if(ret != AI_SUCCESS)
    {
      throw std::runtime_error("ASSIMP failed to export scene " + modelPath_ + " to " + path + ": "
                               + exporter.GetErrorString());
    }
  }

protected:
  void loadScene(unsigned int flags = 0)
  {
    scene_ = importer.ReadFile(modelPath_, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices
                                               | aiProcess_GenNormals | aiProcess_FixInfacingNormals | flags);

    // If the import failed, report it
    if(!scene_)
    {
      throw std::runtime_error(importer.GetErrorString());
    }
  }

  void correctNormalsForAssimp53x(const Eigen::Vector3f & scale)
  {
// ASSIMP version used in Ubuntu 24.04
#if (ASSIMP_MAJOR == 5 && ASSIMP_MINOR == 3 && (ASSIMP_PATCH == 0 || ASSIMP_PATCH == 1))
    // ASSIMP 5.3.[0|1]'s pre-transform step scales vertices but only normalizes existing normals.
    for(unsigned int i = 0; i < scene_->mNumMeshes; ++i)
    {
      aiMesh * mesh = scene_->mMeshes[i];
      if(!mesh->HasNormals())
      {
        continue;
      }

      for(unsigned int j = 0; j < mesh->mNumVertices; ++j)
      {
        aiVector3D normal = mesh->mNormals[j];
        normal.x /= scale.x();
        normal.y /= scale.y();
        normal.z /= scale.z();
        mesh->mNormals[j] = normal.Normalize();
      }
    }
#else
    (void)scale;
#endif
  }

protected:
  // The importer will automatically delete the scene
  Assimp::Importer importer;
  const aiScene * scene_;
  std::string modelPath_;
};

} // namespace mesh_sampling
