
#include<mesh_based_mapping.hpp>

//file with the input point cloud
#include "obj.data" // defines the const char* point_cloud variable

//void readObjVertex(std::basic_istream input_stream,
//                   std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
//                   &landmarks_3d) {

//}

const double pi = std::acos(-1);

void buildHemiSphere(double ro_step, double phi_step,
                     double radius, double central_hole,
                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
                     &landmarks_3d) {

  double x, y, z;

  for (double ro = central_hole; ro < pi / 2; ro += ro_step) {
    z = radius * std::cos(ro);

    for (double phi = -pi; phi < pi; phi += phi_step) {
      x = radius * std::sin(ro) * std::cos(phi);
      y = radius * std::sin(ro) * std::sin(phi);
      printf("%lf,%lf,%lf\n", x, y, z);
    }
  }
}

int main(int, char **) {


  const double focalU = 450;
  const double focalV = 450;
  const double centerU = 400.5;
  const double centerV = 400.5;
  const double dimU = 800;
  const double dimV = 800;
  const double laplaceAlpha = 0.1;
  const unsigned int smoothingIteration = 3;
  const double maxDelta = 0.2;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      input_landmarks_3d;//in camera frame

  buildHemiSphere(0.1, 0.6, 10, 0.3, input_landmarks_3d);

  //build mesh
  //mesh to dmap
  //filter
  //mesh to dmap

//  printf("%s", point_cloud);
  return 0;
}
