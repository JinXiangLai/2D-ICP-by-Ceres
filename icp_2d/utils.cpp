#include "utils.h"

void generate_point_cloud_2d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
  // generate point cloud in a square
  const double resolution = 0.1;
  const double side_len = 5;
  uint8_t color[kOptValNum] = {255, 0, 0};
  // for(double z=0.; z<side_len; z+=(2*resolution)){
  double z = 0.;
  for (double x = 0.; x < side_len; x += resolution) {
    for (double y = 0.; y < side_len; y += resolution) {
      pcl::PointXYZRGB p(color[0], color[1], color[2]);
      p.x = x;
      p.y = y;
      p.z = z;
      pc->push_back(p);
    }
  }
  //}
}

void show_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc,
                      const std::string& title) {
  pcl::visualization::PCLVisualizer viewer(title);
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZRGB>(pc, "", 0);
  while (!viewer.wasStopped()) {
    viewer.spinOnce(1000);
  }
}

void generate_affine_matrix(const double* yaw_xy, Eigen::Matrix4d* T) {
  const double yaw_rad = yaw_xy[0];  // * kDegree2Rad;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  R << cos(yaw_rad), -sin(yaw_rad), sin(yaw_rad), cos(yaw_rad);
  Eigen::Vector2d t(yaw_xy[1], yaw_xy[2]);
  (*T) = Eigen::Matrix4d::Identity();
  (*T).block<2, 2>(0, 0) = R;
  (*T).block<2, 1>(0, 3) = t;
}

void transform_point_cloud(const Eigen::Matrix4d& T,
                           pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr tar) {
  // if (tar->empty()) {
  //   tar = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(
  //       new pcl::PointCloud<pcl::PointXYZRGB>);
  // }
  pcl::transformPointCloud(*src, *tar, T);
}

void find_point_correspondences(const Eigen::Matrix4d& guess_T,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr tar,
                                std::vector<Corres2d>* corres) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src2tar(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  transform_point_cloud(guess_T, src, src2tar);

  // brute force search，kd tree is better
  for (int i = 0; i < src2tar->size(); ++i) {
    double min_dis = DBL_MAX;
    int nearest_id = INT_MIN;
    for (int j = 0; j < tar->size(); ++j) {
      double cur_dis = cal_dist((*src2tar)[i], (*tar)[j]);
      if (cur_dis < min_dis) {
        min_dis = cur_dis;
        nearest_id = j;
      }
    }
    Corres2d correspondence((*src2tar)[i].x, (*src2tar)[i].y,
                            (*tar)[nearest_id].x, (*tar)[nearest_id].y,
                            min_dis);
    corres->push_back(correspondence);
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merge_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
}

double cal_dist(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
  const Eigen::Vector3f diff(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
  return diff.norm();
}

void set_point_cloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, uint8_t r,
                           uint8_t g, uint8_t b) {
  for (auto& p : *pc) {
    p.r = r;
    p.g = g;
    p.b = b;
  }
}