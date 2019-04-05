#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <cmath> 


inline double distance(pcl::PointXYZ &p)
{
   return std::sqrt( p.x* p.x + p.y* p.y + p.z* p.z);
}

int main (int argc, char** argv)
{
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader Reader;
  Reader.read("../Data/000.pcd", *cloud);

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

  double x_l = 180.0;
  double x_u = 380.0;
  double y_l = 100.0;
  double y_u = 280.0;

  double x_range = 640.0;
  double y_range = 360.0;

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);


  double z_min = maxPt.z + 0.5/3.0*(minPt.z - maxPt.z);

  double x_min = minPt.x + x_l/x_range*(maxPt.x - minPt.x);
  double y_min = minPt.y + y_l/y_range*(maxPt.y - minPt.y);

  double x_max = minPt.x + x_u/x_range*(maxPt.x - minPt.x);
  double y_max = minPt.y + y_u/y_range*(maxPt.y - minPt.y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // BoundingBox Point cloud
  for (int i = 0; i < cloud->points.size(); i++)
  { 
    if (cloud->points[i].x >= x_min && cloud->points[i].x <= x_max && cloud->points[i].y >= y_min && cloud->points[i].y <= y_max && cloud->points[i].z >= z_min )
    {
      bb_cloud->push_back(cloud->points[i]);
    }
  }

  std::cerr << "Bounding Box Point cloud data: " << bb_cloud->points.size () << " points" << std::endl;

  pcl::visualization::CloudViewer viewer1 ("Simple Cloud Viewer");
  viewer1.showCloud (bb_cloud);
  while (!viewer1.wasStopped ()) {}

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  while(bb_cloud->points.size() >= 500)
  {
    seg.setInputCloud (bb_cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    plane->points.resize (inliers->indices.size ());
    for (size_t i = 0; i < inliers->indices.size (); ++i) plane->points[i] = bb_cloud->points[inliers->indices[i]];
  
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (plane);
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    for (int i = 0; i < plane->points.size(); i++){
        x += plane->points[i].x;
        y += plane->points[i].y;
        z += plane->points[i].z;
    }
    x = x / plane->points.size();
    y = y / plane->points.size();
    z = z / plane->points.size();
    std::cout<<"Centroid: "<<x<<' '<<y<<" "<<z<<std::endl;
    while (!viewer.wasStopped ()) {}

    // Extract the inliers
    extract.setInputCloud (bb_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    bb_cloud.swap (cloud_f);
  }

  
  return (0);
}