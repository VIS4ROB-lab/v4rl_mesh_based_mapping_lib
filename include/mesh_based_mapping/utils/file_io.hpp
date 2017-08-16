#ifndef MESH_BASED_MAPPING_FILE_IO_H_
#define MESH_BASED_MAPPING_FILE_IO_H_

#include <iostream>
#include <fstream>

namespace mesh_based_mapping {

void saveObj(std::string filepath,
             const VecPoint3f &landmarks_3d,
             const VecTriangle &triangles) {

  std::ofstream ofs;
  ofs.open(filepath, std::ofstream::out);


  for (unsigned int i = 0; i < landmarks_3d.size(); i++) {
    const Eigen::Vector3f &hPoint = landmarks_3d[i];
    ofs << "v "  << hPoint[0] << " " << hPoint[1] << " " << hPoint[2]  << std::endl;
  }

  for (unsigned int i = 0; i < triangles.size(); i++) {

    const Eigen::Vector3i &t = triangles[i];//obj format starts from 1 instead zero
    ofs << "f "  << t[2] + 1 << " ";
    ofs <<  t[1] + 1 << " ";
    ofs <<  t[0] + 1 << std::endl;
  }

  ofs.close();
}

}

#endif
