#include<ros/ros.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/filters/passthrough.h>  
#include<pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include<sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZI PointType;   

ros::Publisher read_pcd_pub;
ros::Publisher pointCloud_pub;

std::string pcd_doc_path;
std::string output_frame_id;
std::string pointCloud_pubTopic;
std::string output_pcd_path;

double pass_x_min, pass_x_max, pass_y_min, pass_y_max, pass_z_min, pass_z_max;  
double voxel_size;
int sor_nearby_number;
double sor_thresh_value;
double x_rotate_value, y_rotate_value, z_rotate_value;
double x_trans_value, y_trans_value, z_trans_value;


// angle to radian
double rad(double d)
{
	return d * 3.1415926 / 180.0;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"read_pcd");
    ros::NodeHandle nh;

    // parameter init
    nh.param<double>("pass_x_min",pass_x_min,1.0);
    nh.param<double>("pass_x_max",pass_x_max,2.0);
    nh.param<double>("pass_y_min",pass_y_min,1.0);
    nh.param<double>("pass_y_max",pass_y_max,2.0);
    nh.param<double>("pass_z_min",pass_z_min,1.0);
    nh.param<double>("pass_z_max",pass_z_max,2.0);
    nh.param<double>("voxel_size",voxel_size,0.01);
    nh.param<int>("sor_nearby_number",sor_nearby_number,30);
    nh.param<double>("sor_thresh_value",sor_thresh_value,1.0);
    nh.param<double>("x_rotate_value",x_rotate_value,0.0);
    nh.param<double>("y_rotate_value",y_rotate_value,0.0);
    nh.param<double>("z_rotate_value",z_rotate_value,0.0);
    nh.param<double>("x_trans_value",x_trans_value,0.0);
    nh.param<double>("y_trans_value",y_trans_value,0.0);
    nh.param<double>("z_trans_value",z_trans_value,0.0);

    nh.param<std::string>("output_frame_id", output_frame_id, "map");
    nh.param<std::string>("pcd_doc_path", pcd_doc_path, "./read_pcd/pcd/test.pcd");
    nh.param<std::string>("pointCloud_pubTopic", pointCloud_pubTopic, "/handle_point");
    nh.param<std::string>("output_pcd_path", output_pcd_path, "./read_pcd/pcd/output.pcd");

    //publish original point cloud and point cloud after precessing
    read_pcd_pub = nh.advertise<sensor_msgs::PointCloud2> ("/input_cloud",10);          
    pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2> (pointCloud_pubTopic,10); 

    // get pcd point cloud
    pcl::PointCloud<PointType>::Ptr pcd_cloud_in (new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (pcd_doc_path, *pcd_cloud_in) == -1)
    {
    PCL_ERROR ("Couldn't read file: %s \n", pcd_doc_path.c_str());
    return (-1);
    }

    sensor_msgs::PointCloud2 input_cloud;
    pcl::toROSMsg(*pcd_cloud_in,input_cloud);
    input_cloud.header.frame_id = output_frame_id;
    read_pcd_pub.publish(input_cloud);


    // process point cloud
    pcl::PointCloud<PointType>::Ptr handle_cloud (new pcl::PointCloud<PointType>);




    // pass-through filter x axis
    pcl::PointCloud<PointType>::Ptr filter_x (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> ptx;
    ptx.setInputCloud(pcd_cloud_in);               
    ptx.setFilterFieldName("x");                     
    ptx.setFilterLimits(pass_x_min, pass_x_max);     
    // ptx.setFilterLimitsNegative(true);           
    ptx.filter(*filter_x);                          
    
    // pass-through filter y axis
    pcl::PointCloud<PointType>::Ptr filter_y (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> pty;
    pty.setInputCloud(filter_x);                  
    pty.setFilterFieldName("y");                    
    pty.setFilterLimits(pass_y_min, pass_y_max);     
    // pty.setFilterLimitsNegative(true);             
    pty.filter(*filter_y);                           

    //  pass-through filter z axis
    pcl::PointCloud<PointType>::Ptr filter_z (new pcl::PointCloud<PointType>);
    pcl::PassThrough<PointType> ptz;
    ptz.setInputCloud(filter_y);                
    ptz.setFilterFieldName("z");                     
    ptz.setFilterLimits(pass_z_min, pass_z_max);   
    // ptz.setFilterLimitsNegative(true);             
    ptz.filter(*filter_z);                          




    // voxcel grid downsampling
    pcl::PointCloud<PointType>::Ptr vox_cloud(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> vox_grid;
    vox_grid.setInputCloud(filter_z);
    vox_grid.setLeafSize(voxel_size, voxel_size, voxel_size); //(m)
    vox_grid.filter(*vox_cloud);


//
//    // RANSAC 
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // create the segmentation object
//    pcl::SACSegmentation<PointType> seg;
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (1);
//
//    seg.setInputCloud (vox_cloud);
//    seg.segment (*inliers, *coefficients);
//
//    if (inliers->indices.size () == 0)
//    {
//        PCL_ERROR ("Could not estimate a planar model for the given pcd file.\n");
//        return (-1);
//    }
//
//    pcl::PointCloud<PointType>::Ptr no_plane (new pcl::PointCloud<PointType>);
//    pcl::ExtractIndices<PointType> extract;
//    extract.setInputCloud(pcd_cloud_in);
//    extract.setIndices(inliers);
//    extract.setNegative(true);
//    extract.filter(*no_plane);

    //  remove outlier
    pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(vox_cloud);
    sor.setMeanK(sor_nearby_number);                            
    sor.setStddevMulThresh(sor_thresh_value);                 
    sor.filter(*filtered_cloud);

    // point cloud rotation and translation
    for(int i = 0; i < filtered_cloud->points.size(); i++) 
    {
        PointType new_point;
        double rx_x,rx_y,rx_z,ry_x,ry_y,ry_z,rz_x,rz_y,rz_z;

        double px = filtered_cloud->points[i].x;
        double py = filtered_cloud->points[i].y;
        double pz = filtered_cloud->points[i].z;
        double pi = filtered_cloud->points[i].intensity;
        
        // rotate around x axis
        rx_x = px;
        rx_y = cos(rad(x_rotate_value))*py + (-sin(rad(x_rotate_value)))*pz;
        rx_z = sin(rad(x_rotate_value))*py + cos(rad(x_rotate_value))*pz;

        //  y axis
        ry_x = cos(rad(y_rotate_value))*rx_x + (-sin(rad(y_rotate_value)))*rx_z;
        ry_y = rx_y;
        ry_z = sin(rad(y_rotate_value))*rx_x + cos(rad(y_rotate_value))*rx_z;

        //  z axis
        rz_x = cos(rad(z_rotate_value))*ry_x + (-sin(rad(z_rotate_value)))*ry_y;
        rz_y = sin(rad(z_rotate_value))*ry_x + cos(rad(z_rotate_value))*ry_y;
        rz_z = ry_z;

        //  translation
        new_point.x = rz_x + x_trans_value;
        new_point.y = rz_y + y_trans_value;
        new_point.z = rz_z + z_trans_value;
        
        new_point.intensity = pi;
        handle_cloud->points.push_back(new_point);
    }
    
    //publish new point cloud

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*handle_cloud,output_cloud);
    output_cloud.header.frame_id = output_frame_id;
    // pointCloud_pub.publish(output_cloud);

    pcl::io::savePCDFileBinary(output_pcd_path, *handle_cloud);    // save new point into pcd

    
    ros::Rate loop_rate(6);
    while (ros::ok())
    {
        read_pcd_pub.publish(input_cloud);
        pointCloud_pub.publish(output_cloud);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}

