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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/package.h>

std::string path = ros::package::getPath("planar_segmentation");
std::string img_path = path+"/Data/000_Color.png";
std::string pcl_path = path+"/Data/000.pcd";

// float P [12] = {-0.14273474, -0.98966773, -0.01358608,  0.0585618 ,-0.10300689,  0.02850545, -0.99427211,  0.51491767, 0.98438629, -0.14051771, -0.10601131,  0.24758666}; 
float P [12] = {614.357421875,  0.0,  310.2319641113281, 0.0, 0.0,   614.494140625,  244.32691955566406, 0.0, 0.0,   0.0,  1.0, 0.0};

void drawBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& plane0, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane1, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane2)
{ 
    std::vector<float> x_arr;
    std::vector<float> y_arr;
    float u;
    float v;
    float w;
   
    for (int i = 0; i < plane0->points.size(); i++){
        u = plane0->points[i].x*P[0]+plane0->points[i].y*P[1]+plane0->points[i].z*P[2]+P[3];
        v = plane0->points[i].x*P[4]+plane0->points[i].y*P[5]+plane0->points[i].z*P[6]+P[7];
        w = plane0->points[i].x*P[8]+plane0->points[i].y*P[9]+plane0->points[i].z*P[10]+P[11];
        x_arr.push_back(u/w);
        y_arr.push_back(v/w);
        std::cout<<u/w<<" "<<v/w<<std::endl;
    }
    for (int i = 0; i < plane1->points.size(); i++){
        u = plane1->points[i].x*P[0]+plane1->points[i].y*P[1]+plane1->points[i].z*P[2]+P[3];
        v = plane1->points[i].x*P[4]+plane1->points[i].y*P[5]+plane1->points[i].z*P[6]+P[7];
        w = plane1->points[i].x*P[8]+plane1->points[i].y*P[9]+plane1->points[i].z*P[10]+P[11];
        x_arr.push_back(u/w);
        y_arr.push_back(v/w);
        std::cout<<u/w<<" "<<v/w<<std::endl;
    }
    for (int i = 0; i < plane2->points.size(); i++){
        u = plane2->points[i].x*P[0]+plane2->points[i].y*P[1]+plane2->points[i].z*P[2]+P[3];
        v = plane2->points[i].x*P[4]+plane2->points[i].y*P[5]+plane2->points[i].z*P[6]+P[7];
        w = plane2->points[i].x*P[8]+plane2->points[i].y*P[9]+plane2->points[i].z*P[10]+P[11];
        x_arr.push_back(u/w);
        y_arr.push_back(v/w);
        std::cout<<u/w<<" "<<v/w<<std::endl;
    }


    cv::Mat image;
    image = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return;
    }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    for(std::vector<float>::size_type i=0; i < x_arr.size(); i++){
      // x_loc=x_arr[i];
      // y_loc=y_arr[i];
      cv::circle(image, cv::Point(x_arr[i],y_arr[i]), 5, CV_RGB(0,255,0),-1);
      }
    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);  
}


inline double distance(pcl::PointXYZ &p)
{
   return std::sqrt( p.x* p.x + p.y* p.y + p.z* p.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::SACSegmentation<pcl::PointXYZ> &seg, pcl::ModelCoefficients::Ptr &coefficients)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return plane;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  
    plane->points.resize (inliers->indices.size ());
    for (size_t i = 0; i < inliers->indices.size (); ++i) plane->points[i] = cloud->points[inliers->indices[i]];
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
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);

    return plane;
}

void view_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ()) {}
}

int main (int argc, char** argv)
{
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader Reader;
  Reader.read(pcl_path, *cloud);

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

  //2D bounding box segmentation

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

  //extract table
  pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud = extract_plane( bb_cloud, seg, coefficients);
  view_pcl(table_cloud);

  //extract plane 0
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane0_cloud = extract_plane( bb_cloud, seg, coefficients);
  view_pcl(plane0_cloud);

  //extract plane 1
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane1_cloud = extract_plane( bb_cloud, seg, coefficients);
  view_pcl(plane1_cloud);

  //extract plane 2
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane2_cloud = extract_plane( bb_cloud, seg, coefficients);
  view_pcl(plane2_cloud);

  drawBoundingBox(plane0_cloud,plane1_cloud,plane2_cloud);





  return (0);
}