#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>

ros::Publisher pub;
float scale = 0;
float tx; float ty; float tz;
float sizeVoxel;
tf::Transform transform_rt;
bool flag = false;

void rt_cb(const geometry_msgs::Transform::Ptr &input)
{
  tf::Transform temp;
  tf::transformMsgToTF( *input, temp);
  transform_rt = temp.inverse();
  //std::cout << "received DO msg" << std::endl;
    flag = true;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{   //std::cout << "received camera msg" << std::endl;
    if(flag)
    {
      // Create a container for the data.
      sensor_msgs::PointCloud2 output;
      
      //Transform Points to scale and translate with relation to previous points
      Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
      transform_1(0, 0) = scale;
      transform_1(1, 1) = scale;
      transform_1(2, 2) = scale;
      transform_1(0, 3) = tx;
      transform_1(1, 3) = ty;
      transform_1(2, 3) = tz;
      
      Eigen::Matrix4f transform_2;
      const tf::Transform temp = transform_rt;
      pcl_ros::transformAsMatrix(temp,transform_2);
      
      //std::cout << "transform1: " << transform_1 << std::endl;
      //std::cout << "transform2: " << transform_2 << std::endl;

      const Eigen::Matrix4f transformTemp = transform_1 * transform_2;
     // std::cout << "transformFinal: " << transformTemp << std::endl;
      pcl_ros::transformPointCloud ( transformTemp, *input, output);

      // Container for original & filtered data
      pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2Ptr cloudFilteredPtr(cloud_filtered);

      // Convert to PCL data type
      pcl_conversions::toPCL(output, *cloud);

      // Perform voxel grid downsampling filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloudPtr);
      sor.setLeafSize(sizeVoxel, sizeVoxel, sizeVoxel);
      sor.filter(*cloudFilteredPtr);

      pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud); // need a boost shared pointer for pcl function inputs

      // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
      pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

      //perform passthrough filtering

      // create a pcl object to hold the passthrough filtered results
      pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);

      // Create the filtering object
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(xyzCloudPtr);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-2.0, 2.0);
      pass.filter(*xyzCloudPtrFiltered);
      pass.setInputCloud(xyzCloudPtrFiltered);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-2.0, 2.0);
      pass.filter(*xyzCloudPtrFiltered);
      pass.setInputCloud(xyzCloudPtrFiltered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-2.0, 2.0);
      pass.filter(*xyzCloudPtrFiltered);

      /*
      pcl::PointXYZRGB maxPt, minPt;
      pcl::getMinMax3D(*xyzCloudPtrFiltered, maxPt, minPt);
      std::cout << "Max x: " << maxPt.x << std::endl;
      std::cout << "Max y: " << maxPt.y << std::endl;
      std::cout << "Max z: " << maxPt.z << std::endl;
      std::cout << "Min x: " << minPt.x << std::endl;
      std::cout << "Min y: " << minPt.y << std::endl;
      std::cout << "Min z: " << minPt.z << std::endl;
      */
      //Convert to PCL2 then to ROS and Publish the data.
      pcl::PCLPointCloud2 outputPCL;
      pcl::toPCLPointCloud2(*xyzCloudPtrFiltered, outputPCL);
      sensor_msgs::PointCloud2 output_ROS;
      pcl_conversions::fromPCL(outputPCL, output_ROS);
      output_ROS.header = input->header; 
      pub.publish (output_ROS);
      flag = false;
    }
}


void loadParameters(ros::NodeHandle nh, std::string parameterLocation, float* saveVar){
  if (nh.getParam(parameterLocation, *saveVar))
  {
    ROS_INFO("Got param: %f", *saveVar);
    ROS_INFO_STREAM("Got param '" << parameterLocation <<"' and its value is " << *saveVar);
  }
  else
  {
    ROS_INFO_STREAM("Failed to get the param '" << parameterLocation <<"' and will probably try again");
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl");
  ros::NodeHandle nh;

  //get scale Parameter
  //if the scale parameter hasn't been set yet, wait one second and try again
  while(scale == 0) {
    loadParameters(nh,"DO/scale",&scale);
    ros::Duration(1).sleep();
  } 

  //get tx,ty,tz
  loadParameters(nh, "DO/tx", &tx);
  loadParameters(nh, "DO/ty", &ty);
  loadParameters(nh, "DO/tz", &tz);

  //get voxel size
  loadParameters(nh, "DO/voxelSize", &sizeVoxel);

  //initialize transformation
  tf::Vector3 origin; origin.setValue(0,0,0);
  tf::Matrix3x3 originR; originR.setValue(1,0,0,0,1,0,0,0,1);
  tf::Quaternion tfqt; originR.getRotation(tfqt);

  transform_rt.setOrigin(origin);
  transform_rt.setRotation(tfqt);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subRT = nh.subscribe("DO/inferenceOut/Transformation",1,rt_cb);
 
  std::string point_topic;
  ros::param::get("~in_point_topic", point_topic);
  ros::Subscriber sub = nh.subscribe (point_topic, 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("my_pcl/downsampling", 1);

  // Spin
  ros::spin ();
}
