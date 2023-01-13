#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

constexpr double kDegree2Rad = M_PI / 180;
constexpr int    kOptValNum = 3;
constexpr double KConverge[kOptValNum] = {1e-5, 1e-5, 1e-5};

struct Corres2d {
  double x1;
  double y1;
  double x2;
  double y2;
  double dist;
  Corres2d() = delete;
  Corres2d(double sx, double sy, double tx, double ty, double dis = 0.)
      : x1(sx), y1(sy), x2(tx), y2(ty), dist(dis) {}
};

void generate_point_cloud_2d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

void show_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc,
                      const std::string& title);

void generate_affine_matrix(const double* yaw_xy, Eigen::Matrix4d* T);

void transform_point_cloud(const Eigen::Matrix4d& T,
                           pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr tar);

void find_point_correspondences(const Eigen::Matrix4d& guess_T,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr tar,
                                std::vector<Corres2d>* corres);

double cal_dist(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2);

void set_point_cloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, uint8_t r,
                           uint8_t g, uint8_t b);