/***********************************************************************************************************************
* @file geometry_detector.cpp
* @brief detect spheres and rectangular prisms on a table and display their height
*
* @author Shaun Sartin
**********************************************************************************************************************/

#include "CloudVisualizer.h"
#include <pcl/common/impl/io.hpp>
#include <pcl/common/geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PCLPointCloud2.h>

#define NUM_COMMAND_ARGS 1


/***********************************************************************************************************************
* @brief Opens a point cloud file
*
* Opens a point cloud file in either PCD or PLY format
*
* @param[out] cloudOut pointer to opened point cloud
* @param[in] filename path and name of input file
* @return false if an error occurred while opening file
* @author Christopher D. McMurrough
**********************************************************************************************************************/
bool openCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut, const char* fileName)
{
    // convert the file name to string
    std::string fileNameStr(fileName);

    // handle various file types
    std::string fileExtension = fileNameStr.substr(fileNameStr.find_last_of(".") + 1);
    if(fileExtension.compare("pcd") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcd file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else if(fileExtension.compare("ply") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcl file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        PCL_ERROR("error while attempting to read unsupported file: %s \n", fileNameStr.c_str());
        return false;
    }
}

/*******************************************************************************************************************//**
 * @brief Locate a plane in the cloud
 *
 * Perform planar segmentation using RANSAC, returning the plane parameters and point indices
 *
 * @param[in] cloudIn pointer to input point cloud
 * @param[out] inliers list containing the point indices of inliers
 * @param[in] distanceThreshold maximum distance of a point to the planar model to be considered an inlier
 * @param[in] maxIterations maximum number of iterations to attempt before returning
 * @return the number of inliers
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
void segmentPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, pcl::PointIndices::Ptr &inliers, double distanceThreshold, int maxIterations)
{
    // store the model coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Create the segmentation object for the planar model and set the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
}

/*******************************************************************************************************************//**
 * @brief Locate a plane in the cloud
 *
 * Perform planar segmentation using RANSAC, returning the plane parameters and point indices
 *
 * @param[in] cloudIn pointer to input point cloud
 * @param[out] inliers list containing the point indices of inliers
 * @param[out] coeffs coefficients for the planar model
 * @param[in] distanceThreshold maximum distance of a point to the planar model to be considered an inlier
 * @param[in] maxIterations maximum number of iterations to attempt before returning
 * @return the number of inliers
 * @author Christopher D. McMurrough and Shaun Sartin
 **********************************************************************************************************************/
void segmentPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients, double distanceThreshold, int maxIterations)
{

    // Create the segmentation object for the planar model and set the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);
}

/*******************************************************************************************************************//**
 * @brief Locate a sphere in the cloud
 *
 * Perform spherical segmentation using RANSAC, returning the sphere parameters and point indices
 *
 * @param[in] cloudIn pointer to input point cloud
 * @param[out] inliers list containing the point indices of inliers
 * @param[out] coeffs coefficients for the spherical model
 * @param[in] distanceThreshold maximum distance of a point to the spherical model to be considered an inlier
 * @param[in] maxIterations maximum number of iterations to attempt before returning
 * @return the number of inliers
 * @author Shaun Sartin
 **********************************************************************************************************************/
void segmentSphere(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudIn, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients, double distanceThreshold, int maxIterations)
{

    // Create the segmentation object for the planar model and set the parameters
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);

    /*
    for (int i = 0; i < coefficients->values.size(); i++)
    {
        cout << "Sphere Coeffs: " << coefficients->values.at(i) << endl;
    }
    */
}

/***********************************************************************************************************************
* @brief convert meters to inches
* @param[in] meters length in meters
* @return returns length in inches
* @author Shaun Sartin
**********************************************************************************************************************/
float metersToInches(float meters)
{
    float inches = meters * 39.3701;
    return inches;
}


/***********************************************************************************************************************
* @brief program entry point
* @param[in] argc number of command line arguments
* @param[in] argv string array of command line arguments
* @return return code (0 for normal termination)
* @author Shaun Sartin
**********************************************************************************************************************/
int main(int argc, char** argv)
{
    // validate and parse the command line arguments
    if(argc != NUM_COMMAND_ARGS + 1)
    {
        std::printf("USAGE: %s <file_name>\n", argv[0]);
        return 0;
    }

    // parse the command line arguments
    char* fileName = argv[1];

    // initialize the cloud viewer
    CloudVisualizer CV("Rendering Window");

    // open the point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    openCloud(cloud, fileName);

    // downsample the cloud using a voxel grid filter
    const float voxelSize = 0.0075;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsize(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxFilter;
    voxFilter.setInputCloud(cloud);
    voxFilter.setLeafSize(static_cast<float>(voxelSize), static_cast<float>(voxelSize), static_cast<float>(voxelSize));
    voxFilter.filter(*cloud_downsize);
    //std::cout << "Points before downsampling: " << cloud->points.size() << std::endl;
    //std::cout << "Points after downsampling: " << cloud_downsize->points.size() << std::endl;

    // segment a plane
    const float distanceThreshold = 0.0254;
    const int maxIterations = 5000;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    segmentPlane(cloud_downsize, inliers, distanceThreshold, maxIterations);
    //std::cout << "Segmentation result: " << inliers->indices.size() << " points" << std::endl;

    // Search through table's points, color them white, and calculate minimum z-distance from camera
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    float minDist;
    for(int i = 0; i < inliers->indices.size(); i++)
    {
        int index = inliers->indices.at(i);

        cloud_downsize->points.at(index).r = 255;
        cloud_downsize->points.at(index).g = 255;
        cloud_downsize->points.at(index).b = 255;

        if (i == 0)
        {
            minDist = cloud_downsize->points.at(index).z;
        }
        else if(minDist > cloud_downsize->points.at(index).z)
        {
            minDist = cloud_downsize->points.at(index).z;
        }

    }



    // Search through all the cloud points and remove the table and anything below it
    pcl::PointIndices::Ptr aboveTableInliers(new pcl::PointIndices);
    for(int i = 0; i < cloud_downsize->points.size(); i++)
    {
        //int index = cloud_downsize->points.at(i);
        if(cloud_downsize->points.at(i).z < minDist)
        {
            aboveTableInliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_downsize);
    extract.setIndices(aboveTableInliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_filtered);

    // Create the euclidian cluster extraction object
    const float clusterDistance = 0.02;
    int minClusterSize = 50;
    int maxClusterSize = 100000;
    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(clusterDistance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);

    // Perform the clustering
    ec.extract(clusterIndices);
    //std::cout << "Clusters identified: " << clusterIndices.size() << std::endl;


    // Calculate normal estimations
    /*
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    //tree->setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch (0.03);
    ne.compute(*cloud_normals);
    */

    //cout << "SIZE OF CLOUD_FILTERED: " << cloud_filtered->points.size() << endl;
    //cout << "SIZE OF CLOUD_NORMALS: " << cloud_normals->points.at(0).data_c[0] << endl;


    pcl::PointXYZRGBA pt_table_centroid;
    pt_table_centroid.x = 0;
    pt_table_centroid.y = 0;
    pt_table_centroid.z = 0;
    for(int j = 0; j < inliers->indices.size(); j++)
    {
        int index = inliers->indices.at(j);

        pt_table_centroid.x = pt_table_centroid.x + cloud_downsize->points.at(index).x;
        pt_table_centroid.y = pt_table_centroid.y + cloud_downsize->points.at(index).y;
        pt_table_centroid.z = pt_table_centroid.z + cloud_downsize->points.at(index).z;
    }
    pt_table_centroid.x = pt_table_centroid.x / inliers->indices.size();
    pt_table_centroid.y = pt_table_centroid.y / inliers->indices.size();
    pt_table_centroid.z = pt_table_centroid.z / inliers->indices.size();

    // Color each cluster
    for(int i = 0; i < clusterIndices.size(); i++)
    {

        int r;
        int g;
        int b;

        switch(i)
        {
            case 0:
                r = 0;
                g = 0;
                b = 255;
                break;
            case 1:
                r = 255;
                g = 0;
                b = 0;
                break;
            case 2:
                r = 0;
                g = 255;
                b = 0;
                break;
            default:
                r = 175;
                g = 255;
                b = 175;
                break;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //cloud_cluster = new pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(*cloud_downsize, clusterIndices.at(i).indices);

        // iterate through the cluster's points and color them all the same
        for(int j = 0; j < clusterIndices.at(i).indices.size(); j++)
        {
            cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).r = r;
            cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).g = g;
            cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).b = b;
            cloud_cluster->push_back(cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)));
        }

        //cout << "Points add to cloud_cluster. Size: " << cloud_cluster->points.size() << endl;

        //cout << "Cluster #" << i << " -- Color: (" << r << ", " << g << ", " << b << ") -- #ofPoints: " << clusterIndices.at(i).indices.size() << endl;

        // Compute the norm for the current cluster and the table
        Eigen::Vector4f clusterNorm_data(0, 0, 0, 0);
        float clusterCurv;
        pcl::computePointNormal(*cloud_filtered, clusterIndices.at(i).indices, clusterNorm_data, clusterCurv);
        //cout << "CLUSTER -- After computePointNormal(): (" << endl << clusterNorm_data.head(4) << ")" << endl;
        //cout << "CLUSTER -- CURVATURE -- :" << clusterCurv << endl;

        Eigen::Vector4f tableNorm_data(0, 0, 0, 0);
        float tableCurv;
        pcl::computePointNormal(*cloud_downsize, inliers->indices, tableNorm_data, tableCurv);
        //cout << "TABLE -- After computePointNormal(): (" << endl << tableNorm_data.head(4) << ")" << endl;
        //cout << "TABLE-- CURVATURE -- :" << tableCurv << endl;

        // Compare the z-value of the norms to determine if the cluster belongs to a ball or a box
        float ballThresh = 0.1;
        if(abs(clusterNorm_data(3) - tableNorm_data(3)) > ballThresh)
        {
            // Cluster is believed to be a ball

            /***********
            // ******NEW APPROACH******
            // Project all points to plane. (use pt_table_centroid for table)
            // Use point which has the greatest distance from its projection to calculate diameter.
                ***********/
            // segment the sphere
            /*
            const float sphereDistanceThreshold = 0.0254;
            const int sphereMaxIterations = 5000;
            pcl::PointIndices::Ptr sphereInliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            segmentSphere(cloud_cluster, sphereInliers, coefficients, sphereDistanceThreshold, sphereMaxIterations);
            //std::cout << "Sphere Segmentation result: " << sphereInliers->indices.size() << " points" << std::endl;
            */

            float maxProjDist = 0;
            for(int j = 0; j < clusterIndices.at(i).indices.size(); j++)
            {
                float pt_dist_x = sqrt(pow(cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).x - pt_table_centroid.x, 2));
                float pt_dist_y = sqrt(pow(cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).y - pt_table_centroid.y, 2));
                float pt_dist_z = sqrt(pow(cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).z - pt_table_centroid.z, 2));

                float projection_x_magnitude = pt_dist_x * tableNorm_data[0];
                float projection_y_magnitude = pt_dist_y * tableNorm_data[1];
                float projection_z_magnitude = pt_dist_z * tableNorm_data[2];

                pcl::PointXYZRGBA pt_projected;
                pt_projected.x = cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).x + projection_x_magnitude;
                pt_projected.y = cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).y + projection_y_magnitude;
                pt_projected.z = cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)).z + projection_z_magnitude;

                float pt_orthog_dist = pcl::geometry::distance(cloud_downsize->points.at(clusterIndices.at(i).indices.at(j)), pt_projected);
                if(pt_orthog_dist > maxProjDist)
                {
                    maxProjDist = pt_orthog_dist;
                }
            }

            cout << "SPHERE - " << metersToInches(maxProjDist) << '"' << endl;

            //CV.addSphere(coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3), r, g, b, 1, "ID", 0);
        }
        else
        {
            // Cluster is believed to be a box
            /*
            const float boxDistanceThreshold = 0.0254;
            const int boxMaxIterations = 5000;
            pcl::PointIndices::Ptr boxInliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            segmentPlane(cloud_cluster, boxInliers, coefficients, boxDistanceThreshold, boxMaxIterations);

            //std::cout << "Box Segmentation result: " << boxInliers->indices.size() << " points" << std::endl;
            cout << "Box -- " << metersToInches(abs(coefficients->values.at(2))) << '"' << endl;
            */

            pcl::PointXYZRGBA pt_cluster_centroid;
            pt_cluster_centroid.x = 0;
            pt_cluster_centroid.y = 0;
            pt_cluster_centroid.z = 0;
            for(int j = 0; j < cloud_cluster->points.size(); j++)
            {
                pt_cluster_centroid.x = pt_cluster_centroid.x + cloud_cluster->points.at(j).x;
                pt_cluster_centroid.y = pt_cluster_centroid.y + cloud_cluster->points.at(j).y;
                pt_cluster_centroid.z = pt_cluster_centroid.z + cloud_cluster->points.at(j).z;
            }
            pt_cluster_centroid.x = pt_cluster_centroid.x / cloud_cluster->points.size();
            pt_cluster_centroid.y = pt_cluster_centroid.y / cloud_cluster->points.size();
            pt_cluster_centroid.z = pt_cluster_centroid.z / cloud_cluster->points.size();

            //pcl::PointXYZRGBA pt_cluster = cloud_cluster->points.at(0);
            //pcl::PointXYZRGBA pt_plane = cloud_downsize->points.at(inliers->indices.at(0));

            float pt_dist_x = sqrt(pow(pt_cluster_centroid.x - pt_table_centroid.x, 2));
            float pt_dist_y = sqrt(pow(pt_cluster_centroid.y - pt_table_centroid.y, 2));
            float pt_dist_z = sqrt(pow(pt_cluster_centroid.z - pt_table_centroid.z, 2));

            float projection_x_magnitude = pt_dist_x * tableNorm_data[0];
            float projection_y_magnitude = pt_dist_y * tableNorm_data[1];
            float projection_z_magnitude = pt_dist_z * tableNorm_data[2];

            pcl::PointXYZRGBA pt_projected;
            pt_projected.x = pt_cluster_centroid.x + projection_x_magnitude;
            pt_projected.y = pt_cluster_centroid.y + projection_y_magnitude;
            pt_projected.z = pt_cluster_centroid.z + projection_z_magnitude;

            /*
            pt_projected.r = 255;
            pt_projected.g = 0;
            pt_projected.b = 0;

            cloud_downsize->points.push_back(pt_projected);
            */

            float pt_orthog_dist = pcl::geometry::distance(pt_cluster_centroid, pt_projected);

            cout << "BOX - " << metersToInches(pt_orthog_dist) << '"' << endl;

            //cout << "CLUSTER -- " << cloud_cluster->points.at(0).z << endl;
            //cout << "TABLE -- " << cloud_downsize->points.at(inliers->indices.at(0)).z << endl;
        }

        // destroy the cluster cloud so that a new one can be instantiated on the next loop iteration
        //cloud_cluster->~PointCloud();

    }

    // render the scene
    CV.addCloud(cloud_downsize);
    CV.addCoordinateFrame(cloud_downsize->sensor_origin_, cloud_downsize->sensor_orientation_);

    /*
    CV.addCloud(cloud_filtered);
    CV.addCoordinateFrame(cloud_filtered->sensor_origin_, cloud_filtered->sensor_orientation_);
    */

    // enter visualization loop
    while(CV.isRunning())
    {
        CV.spin(100);
    }

    // exit program
    return 0;
}
