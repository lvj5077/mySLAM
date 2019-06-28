#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/opencv.hpp>  

// #include <opencv2/core/traits.hpp>
#include <opencv2/opencv.hpp>  
// #include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp> // SIFT

#include <iostream>  
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <string>
#include <opencv2/core/eigen.hpp>
 
using namespace cv;
using namespace std;

// Mat_<double> LinearLSTriangulation(
// 	Point3d u,//homogenous image point (u,v,1)  
// 	Matx34d P,//camera 1 matrix  
// 	Point3d u1,//homogenous image point in 2nd camera  
// 	Matx34d P1//camera 2 matrix  
// 	);

// bool CheckCoherentRotation(cv::Mat_<double>& R);

int main( int argc, char** argv )
{	

	
	// //点云显示框建立
	// pcl::visualization::PCLVisualizer viewer;
	// cout << "正在生成点云，请等待..." << endl;
	
	// //相机标定部分
 
 
 
	// ifstream fin; // 标定所用图像文件的路径 
	// ofstream fout;  // 保存标定结果的文件 
	// int k = 1;
	// int ii = 0;
	
 
	// Matx33d K(1262.958489659621, 0, 562.3792964280647,
	// 	0, 1266.222824039161, 727.8125879762329,
	// 	0, 0, 1);
	// cout << "K="<<K << endl;
	// //sift匹配部分
	// fin.open("sift.txt");
 
	// Mat img_1 = imread("/Users/lingqiujin/work/monoPC/data/20180525163540599.jpg");
	// Mat img_2 = imread("/Users/lingqiujin/work/monoPC/data/20180525163553824.jpg");
	
	// //Create SIFT class pointer
	// cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(0,5,.002);
	// //Detect the keypoints
	// vector<KeyPoint> keypoints_1, keypoints_2;
	// f2d->detect(img_1, keypoints_1);
	// f2d->detect(img_2, keypoints_2);
	// //Calculate descriptors (feature vectors)
	// Mat descriptors_1, descriptors_2;
	// f2d->compute(img_1, keypoints_1, descriptors_1);
	// f2d->compute(img_2, keypoints_2, descriptors_2);
	// //Matching descriptor vector using BFMatcher
	// BFMatcher matcher;
	// vector<DMatch> matches;
	// matcher.match(descriptors_1, descriptors_2, matches);

	// int ptCount = (int)matches.size();
	// cout <<"第"<<k<<"次匹配点云数量为：" << ptCount << endl;
	// Mat p1(ptCount, 2, CV_32F);
	// Mat p2(ptCount, 2, CV_32F);
	// // 把Keypoint转换为Mat
	// //cout << "关键点数目：" << ptCount << endl;
	// Point2f pt;
	// 	for (int i = 0; i < ptCount; i++)
	// 	{
	// 	pt = keypoints_1[matches[i].queryIdx].pt;
	// 	p1.at<float>(i, 0) = pt.x;
	// 	p1.at<float>(i, 1) = pt.y;

	// 	pt = keypoints_2[matches[i].trainIdx].pt;
	// 	p2.at<float>(i, 0) = pt.x;
	// 	p2.at<float>(i, 1) = pt.y;
	// 	}
	// Mat F;
	// F = findFundamentalMat(p1, p2,FM_RANSAC);
	// cout << "基础矩阵为：" << F << endl;
	// //绘制匹配出的关键点
	// Mat img_matches;
	// drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
	// cv::imshow( "Good matches", img_matches );
	// cv::waitKey( 0 );

 
 
 
	// //重建部分
	// Mat_<double> E = Mat(K.t()) * F * Mat(K);
	// SVD svd(E);
	// Matx33d W(0, -1, 0,//HZ 9.13  
	// 	1, 0, 0,
	// 	0, 0, 1);
	// Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19  
	// Mat_<double> t = svd.u.col(2); //u3 
	// 	if (!CheckCoherentRotation(R))
	// 	{
	// 	cout << "resulting rotation is not coherent\n";
	// 	}
	// Matx34d P(1, 0, 0, 0,
	// 	0, 1, 0, 0,
	// 	0, 0, 1, 0);
	// Matx34d P1(R(0, 0), R(0, 1), R(0, 2), t(0),
	// 	R(1, 0), R(1, 1), R(1, 2), t(1),
	// 	R(2, 0), R(2, 1), R(2, 2), t(2));

	// Mat p_1(ptCount, 2, CV_32F);
	// Mat p_2(ptCount, 2, CV_32F);
	// // 把Keypoint转换为Mat
	// for (int i = 0; i < ptCount; i++)
	// {
	// 	pt = keypoints_1[matches[i].queryIdx].pt;
	// 	p_1.at<float>(i, 0) = pt.x;
	// 	p_1.at<float>(i, 1) = pt.y;

	// 	pt = keypoints_2[matches[i].trainIdx].pt;
	// 	p_2.at<float>(i, 0) = pt.x;
	// 	p_2.at<float>(i, 1) = pt.y;
	// }
 
 
	// Mat Kinv;
	// invert(K, Kinv, DECOMP_LU);
	// cout << "这个矩阵为："<<Kinv << endl;
	// //vector<Point3d> u_1, u_2;
	// vector<Point3d> pointcloud;
	// Mat_<double> X_1, X;
	// vector<double> reproj_error;
	// for (size_t i = 0; i < ptCount; i++)
	// {
	// 	Point2f kp = keypoints_1[i].pt;
	// 	Point2f kp1 = keypoints_1[i].pt;
		
	// 	Point3d	u_1(kp.x, kp.y, 1);
	// 	Point3d	u_2(kp1.x, kp1.y, 1);
	// 	//cout << "abc"<<u_1 << u_2<<endl;
	// 	Mat_<double> um_1 = Kinv * Mat_<double>(u_1);
	// 	//	cout <<"ijk"<< um_1 << endl;
	// 	u_1 = Point3d(um_1);
	// 	Mat_<double> um_2 = Kinv * Mat_<double>(u_2);
	// 	u_2 = Point3d(um_2);
	// 	//	cout << "xyz" << u_1 << u_2 << endl;
	// 	X_1 = LinearLSTriangulation(u_1, P, u_2, P1);
	// 	X.push_back(X_1);
	// 	pointcloud.push_back(Point3d(X_1(0), X_1(1), X_1(2)));
	// 	//Mat_<double> xPt_img = K * Mat(P1) * X_1;
	// 	//Point2f xPt_img1(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
	// 	//reproj_error.push_back(norm(xPt_img1 - kp1));

	// }
 

		
	// pcl::PointCloud<pcl::PointXYZ> pointcloud1;
	// pointcloud1.width = ptCount;
	// pointcloud1.height = 1;
	// pointcloud1.is_dense = false;
	// pointcloud1.points.resize(pointcloud1.width * pointcloud1.height);
	// for (size_t i = 0; i < ptCount; i++)
	// {
	// 	pointcloud1[i].x = pointcloud[i].x;
	// 	pointcloud1[i].y = pointcloud[i].y;
	// 	pointcloud1[i].z = pointcloud[i].z;
 
	// }
	// pcl::io::savePCDFile( "pc.pcd", pointcloud1 );

	return 0;
} 
 
 
// //函数定义
// bool CheckCoherentRotation(cv::Mat_<double>& R) {
// 	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
// 		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
// 		return false;
// 	}
// 	return true;
// }
 
// Mat_<double> LinearLSTriangulation(
// 	Point3d u,//homogenous image point (u,v,1)  
// 	Matx34d P,//camera 1 matrix  
// 	Point3d u1,//homogenous image point in 2nd camera  
// 	Matx34d P1//camera 2 matrix  
// 	)
// {
// 	//build A matrix  
// 	Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
// 		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
// 		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
// 		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
// 		);
// 	//build B vector  
// 	Matx41d B(-(u.x*P(2, 3) - P(0, 3)),
// 		-(u.y*P(2, 3) - P(1, 3)),
// 		-(u1.x*P1(2, 3) - P1(0, 3)),
// 		-(u1.y*P1(2, 3) - P1(1, 3)));
// 	//solve for X  
// 	Mat_<double> X;
// 	solve(A, B, X, DECOMP_SVD);
// 	return X;
// }
