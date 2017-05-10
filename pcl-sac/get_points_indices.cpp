#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Help-----
// --------------
void printUsage(const char* progName) {
    std::cout << "\n\nUsage: " << progName << " [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-h           this help\n"
              << "-s           Simple visualisation example\n"
              << "\n\n";
}

void pp_callback(const pcl::visualization::AreaPickingEvent& event,
                 void* viewer_void) {
    std::cout << "hello\n";
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
        *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*>(
             viewer_void);
    std::vector<int> Indices;
    if (event.getPointsIndices(Indices)) {
        std::cout << Indices.size() << std::endl;
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    viewer->registerAreaPickingCallback(pp_callback, (void*)&viewer);

    return (viewer);
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
        printUsage(argv[0]);
        return 0;
    }
    bool simple(false), rgb(false), custom_c(false), normals(false),
        shapes(false), viewports(false), interaction_customization(false);
    if (pcl::console::find_argument(argc, argv, "-s") >= 0) {
        simple = true;
        std::cout << "Simple visualisation example\n";
    } else {
        printUsage(argv[0]);
        return 0;
    }

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Genarating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05) {
        for (float angle(0.0); angle <= 360.0; angle += 5.0) {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
            basic_point.y = sinf(pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb =
                (static_cast<uint32_t>(r) << 16 |
                 static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0) {
            r -= 12;
            g += 12;
        } else {
            g -= 12;
            b += 12;
        }
    }
    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (simple) viewer = simpleVis(basic_cloud_ptr);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}