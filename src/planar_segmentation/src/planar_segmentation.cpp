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
#include <pcl/filters/passthrough.h>
#include <vector>
#include <cmath> 

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>



using namespace std;
using namespace cv;

void ShowManyImages(string title, int nArgs, ...) {
int size;
int i;
int m, n;
int x, y;

// w - Maximum number of images in a row
// h - Maximum number of images in a column
int w, h;

// scale - How much we have to resize the image
float scale;
int max;

// If the number of arguments is lesser than 0 or greater than 12
// return without displaying
if(nArgs <= 0) {
    printf("Number of arguments too small....\n");
    return;
}
else if(nArgs > 14) {
    printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
    return;
}
// Determine the size of the image,
// and the number of rows/cols
// from number of arguments
else if (nArgs == 1) {
    w = h = 1;
    size = 300;
}
else if (nArgs == 2) {
    w = 2; h = 1;
    size = 300;
}
else if (nArgs == 3 || nArgs == 4) {
    w = 2; h = 2;
    size = 300;
}
else if (nArgs == 5 || nArgs == 6) {
    w = 3; h = 2;
    size = 200;
}
else if (nArgs == 7 || nArgs == 8) {
    w = 4; h = 2;
    size = 200;
}
else {
    w = 4; h = 3;
    size = 150;
}

// Create a new 3 channel image
Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);

// Used to get the arguments passed
va_list args;
va_start(args, nArgs);

// Loop for nArgs number of arguments
for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
    // Get the Pointer to the IplImage
    Mat img = va_arg(args, Mat);

    // Check whether it is NULL or not
    // If it is NULL, release the image, and return
    if(img.empty()) {
        printf("Invalid arguments");
        return;
    }

    // Find the width and height of the image
    x = img.cols;
    y = img.rows;

    // Find whether height or width is greater in order to resize the image
    max = (x > y)? x: y;

    // Find the scaling factor to resize the image
    scale = (float) ( (float) max / size );

    // Used to Align the images
    if( i % w == 0 && m!= 20) {
        m = 20;
        n+= 20 + size;
    }

    // Set the image ROI to display the current image
    // Resize the input image and copy the it to the Single Big Image
    Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
    Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
    temp.copyTo(DispImage(ROI));
}

// Create a new window, and show the Single Big Image
namedWindow( title, 1 );
imshow( title, DispImage);
waitKey();

// End the number of arguments
va_end(args);
}

void print_vector(std::vector<double> v)
{
  for (auto a : v) cout<<a<<" ";
    cout<<endl;
}


double dotProduct(const std::vector<double> A, const std::vector<double> B) 
{ 
    assert(A.size() == 3);
    assert(B.size() == 3); 

    double product = 0; 
  
    // Loop for calculate cot product 
    for (int i = 0; i < 3; i++) 
  
        product = product + A[i] + B[i]; 
    return product; 
} 
  
// Function to find 
// cross product of two vector array. 
std::vector<double> crossProduct(const std::vector<double> A, const std::vector<double> B)  
{ 
  assert(A.size() == 3);
  assert(B.size() == 3);

  std::vector<double> ans = A;
  
  ans[0] = A[1] * B[2] - A[2] * B[1]; 
  ans[1] = A[0] * B[2] - A[2] * B[0]; 
  ans[2] = A[0] * B[1] - A[1] * B[0];

  return ans; 
} 

std::vector<double> scalar_product(const double n, const std::vector<double> v)
{
  std::vector<double> ans;
  ans.resize(v.size());
  for (int i = 0; i < v.size(); i++) ans[i] = n * v[i];
  return ans;
}

std::vector<double> vector_add(const std::vector<double> A, const std::vector<double> B)
{
  std::vector<double> ans;
  ans = A;
  for (int i = 0; i < A.size(); i++) ans[i] = A[i] + B[i];
  return ans;
}

std::vector<double> vector_sub(const std::vector<double> A, const std::vector<double> B)
{
  std::vector<double> ans;
  ans = A;
  for (int i = 0; i < A.size(); i++) ans[i] = A[i] - B[i];
  return ans;
}


std::vector<double> calculate_centroid(const std::vector<std::vector<double>> &v, std::vector<double> &d)
{
  std::vector<double> p0;
  // nominator
  std::vector<double> d0;
  std::vector<double> d1;
  std::vector<double> d2;

  d0 = scalar_product(-d[0], crossProduct(v[1], v[2]));
  d1 = scalar_product(-d[1], crossProduct(v[2], v[0]));
  d2 = scalar_product(-d[2], crossProduct(v[0], v[1]));

  std::vector<double> n_v;
  n_v = vector_add(d0, d1);
  n_v = vector_add(n_v,d2);


  double denominator = dotProduct(v[0], crossProduct(v[1], v[2]));
  p0 = scalar_product(1.0/denominator, n_v);
  return p0;
}

std::vector<double> vector_to_wall(const std::vector<double> c, const std::vector<double> p)
{
  double norm = std::sqrt( p[0]* p[0] + p[1]* p[1] + p[2]* p[2]);
  double d = (p[0]*c[0] + p[1]*c[1] + p[2]*c[2] + p[3])/ std::sqrt( p[0]* p[0] + p[1]* p[1] + p[2]* p[2]);
  std::vector<double> ans;
  ans.resize(3);
  for (int i = 0; i < ans.size(); i++) ans[i] = d*p[i]/norm;
  return ans;
}


void drawCountour(cv::Mat &image, std::vector<std::vector<double>> countor, const struct rs2_intrinsics * intrin, cv::Scalar color){
  std::vector<double> x_arr;
  std::vector<double> y_arr;
  for (int i = 0; i < countor.size(); i++)
  {
    float p3[3] = {countor[i][0] + 0.02, countor[i][1], countor[i][2]};
    float p2[2];
    rs2_project_point_to_pixel(p2, intrin, p3);
    x_arr.push_back(p2[0]);
    y_arr.push_back(p2[1]);

  }
  for (std::vector<double>::size_type i=0; i < x_arr.size(); i++){
    int id0 = i % x_arr.size();
    int id1 = (i+1) % x_arr.size();
    cv::line(image, cv::Point(x_arr[id0], y_arr[id0]), cv::Point(x_arr[id1], y_arr[id1]), color, 3, 8, 0);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void drawPoint(cv::Mat &image, std::vector<double> &p, const struct rs2_intrinsics * intrin,  cv::Scalar color)
{ 
  float p3[3] =  {p[0], p[1], p[2]};
  float p2[2];

  rs2_project_point_to_pixel(p2, intrin, p3);
        //image.cols - 
  cv::circle(image, cv::Point(p2[0], p2[1]), 10, color,-1);
}

void drawBoundingBox(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane, const struct rs2_intrinsics * intrin,  cv::Scalar color)
{ 

    std::vector<float> x_arr;
    std::vector<float> y_arr;
    float u;
    float v;
    float w;
   
    for (int i = 0; i < plane->points.size(); i++){
        // u = plane0->points[i].x*P[0]+plane0->points[i].y*P[1]+plane0->points[i].z*P[2]+P[3];
        // v = plane0->points[i].x*P[4]+plane0->points[i].y*P[5]+plane0->points[i].z*P[6]+P[7];
        // w = plane0->points[i].x*P[8]+plane0->points[i].y*P[9]+plane0->points[i].z*P[10]+P[11];

        float p3[3] =  {plane->points[i].x, plane->points[i].y, plane->points[i].z};
        float p2[2];

        rs2_project_point_to_pixel(p2, intrin, p3);
        //image.cols - 
        cv::circle(image, cv::Point(p2[0], p2[1]), 10, color,-1);
        // x_arr.push_back(u/w);
        // y_arr.push_back(v/w);
        // std::cout<<u/w<<" "<<v/w<<std::endl;
        // image.cols -
    }
}


std::vector<std::vector<double>> calculate_countour(std::vector<double> c0, std::vector<double> p1, std::vector<double> p2)
{
  std::vector<std::vector<double>> ans;

  std::vector<double> v0 = vector_to_wall(c0, p1);
  std::vector<double> v1 = vector_to_wall(c0, p2);
  double scale = 1.0;
  std::vector<double> corner0 = vector_add(c0, scalar_product(scale, v0));
  corner0 = vector_add(corner0, scalar_product(scale, v1));
  ans.push_back(corner0);

  corner0 = vector_add(c0, scalar_product(scale, v0));
  corner0 = vector_add(corner0, scalar_product(-scale, v1));
  ans.push_back(corner0);

  corner0 = vector_add(c0, scalar_product(-scale, v0));
  corner0 = vector_add(corner0, scalar_product(-scale, v1));
  ans.push_back(corner0);

  corner0 = vector_add(c0, scalar_product(-scale, v0));
  corner0 = vector_add(corner0, scalar_product(scale, v1));
  ans.push_back(corner0);

  return ans;
}

inline double distance(pcl::PointXYZ &p)
{
   return std::sqrt( p.x* p.x + p.y* p.y + p.z* p.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::SACSegmentation<pcl::PointXYZ> &seg, pcl::ModelCoefficients::Ptr &coefficients, std::vector<double> &centroid)
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
    centroid.resize(3);
    centroid[0] = x / (double)plane->points.size();
    centroid[1] = y / (double)plane->points.size();
    centroid[2] = z / (double)plane->points.size();
    
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
  rs2::pipeline pipe;
  pipe.start();


  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  rs2::frameset frames;
  for(int i = 0; i < 30; i++)
  {
      //Wait for all configured streams to produce a frame
      frames = pipe.wait_for_frames();
  }

  // Get each frame
  rs2::frame color_frame = frames.get_color_frame();

  // Creating OpenCV Matrix from a color image
  Mat image0(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
  Mat image1 = image0.clone();

  // cv::imwrite("small_test.jpg", image0);



  // load data
  // cv::Mat image0;
  // image0 = cv::imread("small_test.jpg", CV_LOAD_IMAGE_COLOR);
  // cv::Mat image1;
  // image1 = cv::imread("small_test.jpg", CV_LOAD_IMAGE_COLOR);


  auto test_color_frame = frames.get_color_frame();
  auto color_profile = test_color_frame.get_profile().as<rs2::video_stream_profile>();
  rs2_intrinsics color_intrin = color_profile.get_intrinsics();

  auto depth = frames.get_depth_frame();
  auto depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
  rs2_intrinsics depth_intrin = depth_profile.get_intrinsics();


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  rs2::points points;
  rs2::pointcloud pc;
  pc.map_to(test_color_frame);
  points = pc.calculate(depth);

  cloud = points_to_pcl(points);

  // load data
  // pcl::PCDReader Reader;
  // Reader.read("small_pcd.pcd", *cloud);

  // pcl::io::savePCDFileASCII ("small_pcd.pcd", *cloud);

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  // view_pcl(cloud);

  

  //2D bounding box segmentation

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*cloud, minPt, maxPt);

  double z_min = 0.3;
  double z_max = 0.7;

  double x_min = minPt.x + 2.3/3.0*(maxPt.x - minPt.x);
  double y_min = minPt.y + 2.25/3.0*(maxPt.y - minPt.y);

  double x_max = minPt.x + 2.5/3.0*(maxPt.x - minPt.x);
  double y_max = minPt.y + 2.5/3.0*(maxPt.y - minPt.y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // BoundingBox Point cloud
  // && cloud->points[i].z >= z_min && cloud->points[i].z <= z_max
  for (int i = 0; i < cloud->points.size(); i++)
  {
  // { (cloud->points[i].x >= x_min && cloud->points[i].x <= x_max && cloud->points[i].y >= y_min && cloud->points[i].y <= y_max  )
    if (cloud->points[i].z <= z_max && cloud->points[i].z >= z_min && cloud->points[i].x >= x_min && cloud->points[i].x <= x_max && cloud->points[i].y >= y_min && cloud->points[i].y <= y_max) 
    {
      bb_cloud->push_back(cloud->points[i]);
    }
  }
  std::cerr << "Bounding Box Point cloud data: " << bb_cloud->points.size () << " points" << std::endl;


  // view_pcl(bb_cloud);
  pcl::getMinMax3D (*bb_cloud, minPt, maxPt);

  // view_pcl(bb_cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC); 
  seg.setDistanceThreshold (0.007);


  std::vector<double> cp;
  //extract table
  pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud = extract_plane( bb_cloud, seg, coefficients, cp);
  // view_pcl(table_cloud);
  // drawBoundingBox(image0, table_cloud, &color_intrin);

  // drawSingleBoundingBox(image, table_cloud, &color_intrin);

  std::vector<std::vector<double>> v;
  std::vector<double> d;
  v.resize(3);
  d.resize(3);
  //extract plane 0

  std::vector<double> c0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane0_cloud = extract_plane( bb_cloud, seg, coefficients, c0);

  v[0].resize(4);
  for (int i = 0; i < 4; i ++) v[0][i] = coefficients->values[i];
  d[0] = coefficients->values[3];  
  // view_pcl(plane0_cloud);
  drawBoundingBox(image0, plane0_cloud, &color_intrin, cv::Scalar(255,0,0));

  //extract plane 1
  std::vector<double> c1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane1_cloud = extract_plane( bb_cloud, seg, coefficients, c1);

  v[1].resize(4);
  for (int i = 0; i < 4; i ++) v[1][i] = coefficients->values[i];
  d[1] = coefficients->values[3];
  // view_pcl(plane1_cloud);
  drawBoundingBox(image0, plane1_cloud, &color_intrin, cv::Scalar(0,255,0));

  //extract plane 2
  std::vector<double> c2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane2_cloud = extract_plane( bb_cloud, seg, coefficients, c2);

  v[2].resize(4);
  for (int i = 0; i < 4; i ++) v[2][i] = coefficients->values[i];
  d[2] = coefficients->values[3];
  // view_pcl(plane2_cloud);

  drawBoundingBox(image0, plane2_cloud, &color_intrin, cv::Scalar(0,0,255));


  // std::vector<double> p0 = calculate_centroid(v, d); 

  // // drawPoint(image1, p0, &color_intrin, cv::Scalar(255,255,255));
  // drawPoint(image1, c0, &color_intrin, cv::Scalar(255,255,255));
  //  print_vector(c0);
  // // drawPoint(image1, c1, &color_intrin, cv::Scalar(255,255,255));
  // // drawPoint(image1, c2, &color_intrin, cv::Scalar(255,255,255));

  std::vector<std::vector<double>> countor_0 = calculate_countour(c0, v[1], v[2]);
  std::vector<std::vector<double>> countor_1 = calculate_countour(c1, v[0], v[2]);
  std::vector<std::vector<double>> countor_2 = calculate_countour(c2, v[0], v[1]);

  // cout<<countor_0.size()<<endl;
  // for (int i = 0; i < countor_0.size(); i++) 
  //   {
  //     print_vector(countor_0[i]);
  //     drawPoint(image1, countor_0[i], &color_intrin, cv::Scalar(0,0,0));
  //   }

  drawCountour( image1, countor_0, &color_intrin, cv::Scalar(255,0,0));
  drawCountour( image1, countor_1, &color_intrin, cv::Scalar(0,255,0));
  drawCountour( image1, countor_2, &color_intrin, cv::Scalar(0,0,255));


  ShowManyImages("Image", 2, image0, image1); 

  return (0);
}