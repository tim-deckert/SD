#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "voxel_grid.h"


int voxelfilter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered)
{
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloudFiltered);

  std::cerr << "PointCloud after filtering: " << cloudFiltered->width * cloudFiltered->height
      << " data points (" << pcl::getFieldsList (*cloudFiltered) << ")." << "\n";



  return (0);
}
