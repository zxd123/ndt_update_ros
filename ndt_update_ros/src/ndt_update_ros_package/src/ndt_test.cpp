//
// Created by zxd on 20-10-16.
//
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>

#include <ndt_update.h>
//#include <ndt_update.hpp>
#include <voxel_grid_covariance_update.h>
#include <voxel_grid_covariance_update.hpp>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std :: literals :: chrono_literals;

int
main (int argc, char** argv)
{
    // Initializing Normal Distributions Transform (NDT).
    pcl_update::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_ori;
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01);
    ndt_ori.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    ndt_ori.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);
    ndt_ori.setResolution(1.0);
    //记录前一帧位姿
    Eigen::Matrix4f T_last=ndt.getFinalTransformation();
    Eigen::Matrix4f T_last_ori=ndt_ori.getFinalTransformation();
    //记录当前位姿
    Eigen::Matrix4f T_now=ndt.getFinalTransformation();
    Eigen::Matrix4f T_now_ori=ndt_ori.getFinalTransformation();
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(60);
    ndt_ori.setMaximumIterations(60);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_update(new pcl::PointCloud<pcl::PointXYZ>);
    // Loading first scan of room.
    if(target_cloud->empty()) {
      // /media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/
      if (pcl::io::loadPCDFile<pcl::PointXYZ>("/media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/1.pcd", *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
      }
      std::cout << "Loaded " << target_cloud->size() << " data points from 1.pcd" << std::endl;
    }
    *target_cloud_update = *target_cloud;
    // 设置初始要建立ndt地图的点云.
    ndt.setInputTarget(target_cloud);
    ndt_ori.setInputTarget(target_cloud);

    double ori_sum=0,diff_sum=0;
    for(int i=2;i<51;i++) {
        // Loading second scan of room from new perspective.
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // "/media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/"
        std::string number="/media/zxd/60787A51787A25C6/pcl_ndt/PointCloud/bin2pcd/velodyne/pcd/1025/"+std::to_string(i)+".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(number, *input_cloud) == -1) {
            PCL_ERROR ("Couldn't read file ");
            //return (-1);
            break;
        }
        std::cout << "Loaded " << input_cloud->size() << " data points from "+number << std::endl;

        // Filtering input scan to roughly 10% of original size to increase speed of registration.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud(input_cloud);
        approximate_voxel_filter.filter(*filtered_cloud);
        std::cout << "Filtered cloud contains " << filtered_cloud->size()
                  << " data points from room_scan2.pcd" << std::endl;
//
        // Setting point cloud to be aligned.
        ndt.setInputSource(filtered_cloud);
        ndt_ori.setInputSource(filtered_cloud);
        Eigen::Matrix4f init_guess = T_now*T_last.inverse()*T_now;
        Eigen::Matrix4f init_guess_ori = T_now_ori*T_last_ori.inverse()*T_now_ori;
        // Calculating required rigid transform to align the input cloud to the target cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud, init_guess);
        ndt_ori.align(*output_cloud_ori, init_guess_ori);
        T_last=T_now;
        T_last_ori=T_now_ori;
        T_now=ndt.getFinalTransformation();
        T_now_ori=ndt_ori.getFinalTransformation();
        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
                  << " update_score: " << ndt.getFitnessScore() << std::endl;
        std::cout << "Normal Distributions Transform has converged:" << ndt_ori.hasConverged()
                  << " ori_score: " << ndt_ori.getFitnessScore() << std::endl;

        diff_sum += ndt.getFitnessScore() - ndt_ori.getFitnessScore();
        ori_sum += ndt_ori.getFitnessScore();

        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
        pcl::transformPointCloud(*input_cloud, *output_cloud_ori, ndt_ori.getFinalTransformation());
        // 更新ndt地图
        start=clock();		//程序开始计时
        ndt.updateInputTarget(output_cloud);
        end=clock();		//程序结束计时
        double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        std::cout<<"update_Total time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位
        *target_cloud = *target_cloud + *output_cloud_ori;
        *target_cloud_update = *target_cloud_update + *output_cloud;

        start=clock();		//程序开始计时
        ndt_ori.setInputTarget(target_cloud);
        end=clock();		//程序结束计时
        endtime=(double)(end-start)/CLOCKS_PER_SEC;
        std::cout<<"ori_Total time:"<<endtime*1000<<"ms"<<std::endl;	//ms为单位
    }
    cout << "精度相差的百分比 = " << diff_sum / ori_sum *100 <<"%"<<endl;
    // Saving result cloud.
    pcl::io::savePCDFileASCII ("result_ori.pcd", *target_cloud);
    pcl::io::savePCDFileASCII ("result_update.pcd", *target_cloud_update);
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//            output_color (output_cloud, 0, 255, 0);
//    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
//    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                    1, "output cloud");

    // Coloring and visualizing transformed input cloud (green).
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//            result_color (result_cloud, 0, 0, 255);
//    viewer_final->addPointCloud<pcl::PointXYZ> (result_cloud, result_color, "result cloud");
//    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                    1, "result cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}
