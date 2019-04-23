#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/cloud_viewer.h>


#include <iostream>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;
using namespace pcl;

struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale, depthL, depthH,height,width,exp;
};

struct SR4kFRAME
{
    int frameID; 
    cv::Mat rgb, depthXYZ, z;
    cv::Mat desp;
    vector<cv::KeyPoint> kp;
    cv::Mat pose;
};

class slamBase{
	private:
		CAMERA_INTRINSIC_PARAMETERS C;
		// double depthL = 0.18;
		// double depthH = 5.0;

	public:
		slamBase();
		~slamBase();
		void setCamera(CAMERA_INTRINSIC_PARAMETERS inC);
		CAMERA_INTRINSIC_PARAMETERS getCamera();

		SR4kFRAME readSRFrame( string inFileName);

		void find4kMatches(SR4kFRAME & frame1, SR4kFRAME & frame2, 
			vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2, double* overlapRate = NULL );

		
	    cv::Point3f point2dTo3d( cv::Point2f& point, double& d, CAMERA_INTRINSIC_PARAMETERS& camera );
	    double reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t );
	    double reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & T );
	    void rotMtoRPY(Mat &mat_r, float &roll,float &pitch,float &yaw);
	    Mat eulerAnglesToRotationMatrix(float roll,float pitch,float yaw);

	    pcl::PointCloud<pcl::PointXYZ> cvPtsToPCL(vector<Point3f> &p_XYZs);
	    vector<Point3f> imagToCVpt( Mat depth, CAMERA_INTRINSIC_PARAMETERS& camera );
	    Eigen::Isometry3d cvTtoEigenT( Mat cv44T);
};
