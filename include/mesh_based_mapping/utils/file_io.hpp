#ifndef MESH_BASED_MAPPING_FILE_IO_H_
#define MESH_BASED_MAPPING_FILE_IO_H_

namespace mesh_based_mapping {


/**

*/
void writeCSV(std::string filename, cv::Mat m) {
  cv::Formatter const *c_formatter(cv::Formatter::get("CSV"));
  std::ofstream myfile;
  myfile.open(filename.c_str());
  c_formatter->write(myfile, m);
  myfile.close();
}




/**

*/
void saveObj(std::string filepath, VecPoint3f &points3d,
             std::vector<GEOM_FADE2D::Point2> points2D,
             std::vector<GEOM_FADE2D::Triangle2 *> triangles, std::vector<bool> &blacklist) {
  std::ofstream ofs;
  ofs.open(filepath, std::ofstream::out);


  for (unsigned int i = 0; i < points3d.size(); i++) {
    Eigen::Vector3f hPoint = points3d[i];
    ofs << "v "  << hPoint[0] << " " << hPoint[1] << " " << hPoint[2]  << std::endl;
  }

  for (unsigned int i = 0; i < points2D.size(); i++) {
    GEOM_FADE2D::Point2 hPoint = points2D[i];
    ofs << "vt "  << hPoint.x() / 752 << " " << hPoint.y() / 480 << std::endl;
  }

  for (unsigned int i = 0; i < triangles.size(); i++) {
    if (blacklist[i]) {
      continue;
    }

    GEOM_FADE2D::Triangle2 *t = triangles[i];
    ofs << "f "  << t->getCorner(2)->getCustomIndex() + 1 << "/" << t->getCorner(
          2)->getCustomIndex() + 1 << " ";
    ofs << t->getCorner(1)->getCustomIndex() + 1 << "/" << t->getCorner(
          1)->getCustomIndex() + 1 << " ";
    ofs << t->getCorner(0)->getCustomIndex() + 1 << "/" << t->getCorner(
          0)->getCustomIndex() + 1 << std::endl;
  }

  ofs.close();
}

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
