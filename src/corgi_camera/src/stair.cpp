#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <iomanip>

class StairDistTest
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher  dist_pub_, cloud_pub_, marker_pub_, plane_pub_;

  pcl::VoxelGrid<pcl::PointXYZ> vg_;
  pcl::PassThrough<pcl::PointXYZ> pass_x_, pass_y_, pass_z_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_;
  pcl::ExtractIndices<pcl::PointXYZ> extract_;

  /* ROI 參數（m）*/
  const float X_MIN = 0.0f,  X_MAX = 2.5f;
  const float Y_MIN = -0.5f, Y_MAX = 0.5f;
  const float Z_MIN = -0.5f, Z_MAX = -0.1f;

public:
  explicit StairDistTest(const std::string& cloud_topic)
  {
    sub_  = nh_.subscribe(cloud_topic, 1, &StairDistTest::cb, this);
    dist_pub_   = nh_.advertise<std_msgs::Float32>("stair_distance_test", 1);
    cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("roi_marker", 1);
    plane_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);

    /* 2 cm 下采樣 */
    vg_.setLeafSize(0.02f, 0.02f, 0.02f);

    /* ROI PassThrough */
    pass_x_.setFilterFieldName("x"); pass_x_.setFilterLimits(X_MIN, X_MAX);
    pass_y_.setFilterFieldName("y"); pass_y_.setFilterLimits(Y_MIN, Y_MAX);
    pass_z_.setFilterFieldName("z"); pass_z_.setFilterLimits(Z_MIN, Z_MAX);

    /* RANSAC 設定：垂直平面（法向≈±X）*/
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);            
    seg_.setDistanceThreshold(0.015);                // 1.5 cm 內視為平面點
    seg_.setAxis(Eigen::Vector3f::UnitX());          // X 軸
    seg_.setEpsAngle(15.0 * M_PI / 180.0);           // ±15°
    seg_.setMaxIterations(200);
    extract_.setNegative(false);                     // 只取內點
  }

  void cb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    /* PointCloud2 → PCL */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    /* 1. VoxelGrid 下採樣 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg_.setInputCloud(cloud); vg_.filter(*ds);

    /* 2. ROI */
    pass_x_.setInputCloud(ds); pass_x_.filter(*ds);
    pass_y_.setInputCloud(ds); pass_y_.filter(*ds);
    pass_z_.setInputCloud(ds); pass_z_.filter(*ds);
    if (ds->empty()) return;

    /* ---------- A) 發布 ROI 點雲 ---------- */
    sensor_msgs::PointCloud2 roi_msg;
    pcl::toROSMsg(*ds, roi_msg);
    roi_msg.header = msg->header;
    cloud_pub_.publish(roi_msg);

    /* ---------- B) 發布 ROI 立方體 Marker ---------- */
    visualization_msgs::Marker cube;
    cube.header = msg->header;
    cube.ns = "roi"; cube.id = 0;
    cube.type = visualization_msgs::Marker::CUBE; cube.action = visualization_msgs::Marker::ADD;
    cube.pose.position.x = (X_MIN + X_MAX) / 2.0f;
    cube.pose.position.y = (Y_MIN + Y_MAX) / 2.0f;
    cube.pose.position.z = (Z_MIN + Z_MAX) / 2.0f;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = X_MAX - X_MIN; cube.scale.y = Y_MAX - Y_MIN; cube.scale.z = Z_MAX - Z_MIN;
    cube.color.r = 0.0f; cube.color.g = 0.8f; cube.color.b = 1.0f; cube.color.a = 0.3f;
    marker_pub_.publish(cube);

    /* ---------- C) RANSAC 找垂直平面 ---------- */
    pcl::ModelCoefficients coeff;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg_.setInputCloud(ds);
    seg_.segment(*inliers, coeff);
    if (inliers->indices.size() < 150) return;      // 內點不足 → 不可信

    /* 擷取內點 */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract_.setInputCloud(ds); extract_.setIndices(inliers);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    extract_.filter(*tmp);
    plane->reserve(tmp->size());
    for (const auto& p : *tmp) {
      pcl::PointXYZRGB pt;
      pt.x = p.x; pt.y = p.y; pt.z = p.z;
      pt.r = 255; pt.g = 0; pt.b = 0;  // 紅色
      plane->push_back(pt);
    }

    /* 發布紅點雲 */
    sensor_msgs::PointCloud2 plane_msg;
    pcl::toROSMsg(*plane, plane_msg);
    plane_msg.header = msg->header;
    plane_pub_.publish(plane_msg);

    /* ---------- D) 最近距離（平面到相機） ---------- */
    // 平面方程 ax + by + cz + d = 0
    float a = coeff.values[0], b = coeff.values[1],
          c = coeff.values[2], d = coeff.values[3];
    float dist = std::abs(d) / std::sqrt(a*a + b*b + c*c);

    std_msgs::Float32 out; out.data = dist;
    dist_pub_.publish(out);
    std::cout << std::fixed << std::setprecision(3)
              << "[StairDist] distance = " << dist << " m" << std::endl;
  }
};

/* ====================== main ====================== */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stair_distance_test_node");
  ros::NodeHandle pnh("~");

  std::string cloud_topic;
  pnh.param<std::string>("cloud_topic",
                         cloud_topic,
                         "/zedxm/zed_node/point_cloud/cloud_registered");  // RAW 相機框架雲

  StairDistTest sdt(cloud_topic);
  ros::spin();
  return 0;
}