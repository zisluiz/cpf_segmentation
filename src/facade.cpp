#include <segmentation/segmentation.hpp>
#include <segmentation/configprop.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace APC;

class Facade
{
    public:
        Facade(std::string configFilePath);
        pcl::PointCloud<pcl::PointXYZL>::Ptr segmentImage(pcl::PointCloud<PointT>::Ptr pointCloudIn, bool showDebug);
    private:
        Segmentation seg;
        Config config;
};

Facade::Facade(std::string configFilePath)
{
    ConfigProperties *configProp = new ConfigProperties(configFilePath);
    config.noise_threshold = std::stod(configProp->config["noise_threshold"]);
    config.voxel_resolution = std::stof(configProp->config["voxel_resolution"]);
    config.seed_resolution = std::stof(configProp->config["seed_resolution"]);
    config.min_plane_area = std::stof(configProp->config["min_plane_area"]);
    config.max_curvature = std::stod(configProp->config["max_curvature"]);

    config.color_importance = std::stof(configProp->config["color_importance"]);
    config.spatial_importance = std::stof(configProp->config["spatial_importance"]);
    config.use_single_cam_transform = std::stoi(configProp->config["use_single_cam_transform"]);
    config.use_supervoxel_refinement = std::stoi(configProp->config["use_supervoxel_refinement"]);
    config.use_random_sampling = std::stoi(configProp->config["use_random_sampling"]);
    config.min_inliers_per_plane = std::stoi(configProp->config["min_inliers_per_plane"]);
    config.max_num_iterations = std::stoi(configProp->config["max_num_iterations"]);
    config.normal_importance = std::stof(configProp->config["normal_importance"]);
    config.smooth_cost = std::stod(configProp->config["smooth_cost"]);
    config.gc_scale = std::stod(configProp->config["gc_scale"]);

    seg.setConfig(config);
    delete configProp; 
}

pcl::PointCloud<pcl::PointXYZL>::Ptr Facade::segmentImage(pcl::PointCloud<PointT>::Ptr pointCloudIn, bool showDebug) {
  std::vector< int > index;
  if (!pointCloudIn->is_dense){
    if (showDebug)
        PCL_WARN("Point data not dense, eating NaNs\n");
    pcl::removeNaNFromPointCloud<PointT>(*pointCloudIn, *pointCloudIn,index);
    if (showDebug)
        PCL_INFO ("Done making cloud\n");
  }
  std::cerr << "Number of points: " << pointCloudIn->size() << std::endl;

  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(pointCloudIn);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter(*cloud_filtered);

  if (showDebug)
    std::cerr << "Number of points after filtered " << cloud_filtered->size() << std::endl;

  seg.setPointCloud(pointCloudIn);

  seg.doSegmentation();

  pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr;

  segmented_cloud_ptr=seg.getSegmentedPointCloud();

  return segmented_cloud_ptr;
}

extern "C"
{   
    Facade* Facade_new(const char* configFilePath) {return new Facade(std::string(configFilePath));}
    pcl::PointCloud<pcl::PointXYZL>::Ptr Facade_segmentImage(Facade* facade, pcl::PointCloud<PointT>::Ptr pointCloudIn, bool showDebug) {
        return facade->segmentImage(pointCloudIn, showDebug);
    }   
}