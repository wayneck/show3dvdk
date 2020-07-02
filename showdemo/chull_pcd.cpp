#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
 
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
 
  reader.read (argv[1], *cloud);
  
  //建立直通滤波器，消除杂散的NaN点
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);   //设置输入点云
  pass.setFilterFieldName ("x");//设置分隔字段为z坐标
  pass.setFilterLimits (0, 1.1);//设置分割阈值范围，z轴上不在该范围的点过滤掉
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;
 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers指针存储点云分割后的结果
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;//创建分割对象
  // 可选设置
  seg.setOptimizeCoefficients (true);     //设置优化系数，该参数为可选设置
  // 必须设置
  seg.setModelType (pcl::SACMODEL_PLANE); //设置分割模型为SACMODEL_PLANE 平面模型
  seg.setMethodType (pcl::SAC_RANSAC);    //设置采样一致性估计方法模型为SAC_RANSAC
  //seg.setDistanceThreshold (0.01);        //设置距离阈值为0.01， 即与估计平面模型的距离小于0.01m的点都为内点inliers
  seg.setDistanceThreshold (0.5);
  seg.setInputCloud (cloud_filtered);     //设置输入点云为滤波后的点云
  seg.segment (*inliers, *coefficients);  //分割结果：平面模型
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;
 
  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj; //创建点云投影滤波对象
  proj.setModelType (pcl::SACMODEL_PLANE); //设置投影模型为SACMODEL_PLANE
  proj.setInputCloud (cloud_filtered);     //设置输入点云为滤波后的点云
  proj.setModelCoefficients (coefficients);//将估计得到的平面模型coefficients参数设置为投影平面模型系数
  proj.filter (*cloud_projected);          //将滤波后的点云投影到平面模型中得到投影后的点云cloud_projected
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;
 
  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;//创建多边形提取对象
  chull.setInputCloud (cloud_projected);//设置输入点云为投影后的点云cloud_projected
  chull.setAlpha (0.1);                 //设置Alpha值为0.1
  chull.reconstruct (*cloud_hull);      //重建提取创建凹多边形
 
  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;
 
  pcl::PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
 
//   pcl::visualization::PCLVisualizer::Ptr viewer0(new pcl::visualization::PCLVisualizer("hull"));
//   viewer0->addPointCloud(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 0, 255), "cloud");
//   viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
//   cout << "cloud click q key to quit the visualizer and continue！！" << endl;
//   viewer0->spin();
 
  pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("hull"));
  viewer1->addPointCloud(cloud_hull, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_hull, 0, 255, 0), "cloud_hull");
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_hull");
  cout << "cloud_hull click q key to quit the visualizer and continue！！" << endl;
  viewer1->spin();
 
  return (0);
}