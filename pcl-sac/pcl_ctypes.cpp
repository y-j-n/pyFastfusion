#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

extern "C" void lib_process_cloud(
    char *ach_cloud_in, char *ach_planes_dir,
    float leafLen,
    float sacDistThresh,
    float fpercentLeftSkip, // e.g. 0.3 -> extraction stops when less than 30% of nr_points are left
    float clusterTol // not used for planar seg...
) {
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream ss_cloud_in;

    //  ss_cloud_in << "../table_scene_lms400.pcd";
    //  float leafLen = 0.01f, sacDistThresh = 0.02f, clusterTol = 0.02f;

    //  ss_cloud_in << "./cloud_snap.pcd";
    //  float leafLen = 0.05f, sacDistThresh = 0.10f, clusterTol = 0.10f;

    //  std::cout << "input: " << ss_cloud_in.str() << std::endl; //*
    //  reader.read(ss_cloud_in.str(), *cloud);
    reader.read(ach_cloud_in, *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size()
              << " data points." << std::endl;  //*

    // Create the filtering object: downsample the dataset using a leaf size of
    // 1cm (refactored into leafLen)
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafLen, leafLen, leafLen);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: "
              << cloud_filtered->points.size() << " data points."
              << std::endl;  //*


    // SAC-based planar extraction
    if (1) {//!!!!!!!!!!!!!!!!!!!!!!!
        // Create the segmentation object for the planar model and set all the
        // parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
            new pcl::PointCloud<pcl::PointXYZ>());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(sacDistThresh);

        int i = 0, nr_points = (int)cloud_filtered->points.size();

        std::stringstream ss_file_abdc;
        ss_file_abdc << ach_planes_dir << "/models.txt";
        // http://stackoverflow.com/questions/1374468/stringstream-string-and-char-conversion-confusion
        std::string tmp_str = ss_file_abdc.str();
        FILE *fd_abcd = std::fopen(tmp_str.c_str(), "w");

        while (cloud_filtered->points.size() > fpercentLeftSkip * nr_points) {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cout
                    << "Could not estimate a planar model for the given dataset."
                    << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);

            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            // http://pointclouds.org/documentation/tutorials/planar_segmentation.php

            // save model params to models.txt
            if (i == 0) {
                std::cout << "<planar component file> <# of points> <A> <B> <C> <D>"
                          << std::endl;
            }
            std::stringstream ss_line_abcd;
            ss_line_abcd << i << ".pcd " << cloud_plane->points.size() << " "
                         << coefficients->values[0] << " "
                         << coefficients->values[1] << " "
                         << coefficients->values[2] << " "
                         << coefficients->values[3];
            tmp_str = ss_line_abcd.str();
            std::cout << tmp_str << std::endl;
            fprintf(fd_abcd, "%s\n", tmp_str.c_str());

            // save planar componenet points to *.pcd
            std::stringstream ss;
            ss << ach_planes_dir << "/" << i << ".pcd";
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>(ss.str(), *cloud_plane, false);
            i++;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }
        std::fclose(fd_abcd);
    }//!!!!!!!!!!!!!!!!!!!!!!!!

    // don't need this for just finding planes...
    if (0) {// EuclideanClusterExtraction!!!!!!!!!!!!!!!!!!!!!!!
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(clusterTol);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);  //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud (Cluster): " << cloud_cluster->points.size()
                      << " points." << std::endl;
            std::stringstream ss;
            ss << ach_planes_dir << "/" << "cloud_cluster_" << j << ".pcd";
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);

            j++;
        }
    }//!!!!!!!!!!!!!!!!!!!!!!
}