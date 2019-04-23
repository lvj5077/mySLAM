#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <stdlib.h>   


#include "slamBase.h"
#include "pose_estimation.h"


 

using namespace std;
using namespace pcl;

#include "optimizeG2O.h"

// struct myPoseAtoB
// {
//     int posA_id = 0;
//     int posB_id = 0;
//     Mat T_ab = cv::Mat::eye(4,4,CV_64F);
    
// };

const int startIdx=3;
const int endIdx=7;

void frame2framePose(int firstF_id,int secondF_id,vector<myPoseAtoB> &poseChain,vector<SR4kFRAME> &frames, 
    pose_estimation myVO_SR4k,slamBase myBase_SR4k );

int main( int argc, char** argv )
{


	double depthL = 0.80;
	double depthH = 8.000;

    CAMERA_INTRINSIC_PARAMETERS C_sr4k;

    C_sr4k.cx = 88.5320;
    C_sr4k.cy = 72.5102;
    C_sr4k.fx = 222.6132;
    C_sr4k.fy = 225.6439;
    C_sr4k.scale = 1000;
    C_sr4k.depthL = 0.180;
    C_sr4k.depthH = 7.000;
    C_sr4k.height = 144;
    C_sr4k.width = 176;
    C_sr4k.exp = 8;//50;


    std::vector<myPoseAtoB> poseChain;

    pose_estimation myVO_SR4k; 
    slamBase myBase_SR4k; 

    optimizeG2O myOpt_SR4k; 

    myBase_SR4k.setCamera(C_sr4k);

    CAMERA_INTRINSIC_PARAMETERS C = myBase_SR4k.getCamera();
    double camera_matrix_data_4k[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix_4k( 3, 3, CV_64F, camera_matrix_data_4k );




    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_all (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<SR4kFRAME> frames;
    std::vector<SR4kFRAME> keyframes;
    // string data_path = "/Users/lingqiujin/Data/RV_Data2/d1_";
    string data_path = "/Users/lingqiujin/Data/test4K/dt_";

    // endIdx = endIdx-1; // frame 2 frame // make sure last frame is valid
    for (int idx=startIdx;idx<endIdx;idx++){

        std::stringstream ss;

        int firstF_id = idx;
        int secondF_id = idx+1;

        ss << data_path <<std::setw(4) << std::setfill('0') << firstF_id <<".dat";
        string firstF = ss.str();

        ss= std::stringstream();
        ss << data_path <<std::setw(4) << std::setfill('0') << secondF_id <<".dat";
        string secondF = ss.str();


        // SR4kFRAME f1_4k = myBase_SR4k.readSRFrame(firstF);
        // SR4kFRAME f2_4k = myBase_SR4k.readSRFrame(secondF);
        SR4kFRAME f1_4k; 
        SR4kFRAME f2_4k;
        if ( idx == startIdx ){
            f1_4k = myBase_SR4k.readSRFrame(firstF);
            f1_4k.frameID = firstF_id;
            f1_4k.pose = cv::Mat::eye(4,4,CV_64F);
            frames.push_back(f1_4k);


            myPoseAtoB fixedAtoA;
            fixedAtoA.posA_id = firstF_id;
            fixedAtoA.posB_id = firstF_id;
            fixedAtoA.T_ab = cv::Mat::eye(4,4,CV_64F);
            poseChain.push_back(fixedAtoA);

        }
        // frame2 is always the new frame
        f2_4k = myBase_SR4k.readSRFrame(secondF);
        f2_4k.frameID = secondF_id;
        frames.push_back(f2_4k); 
        
        frame2framePose(firstF_id,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k);

        srand (time(NULL));
        int randomCheckID = rand()%frames.size()+ startIdx; 
        cout << randomCheckID<<endl;
        frame2framePose(randomCheckID,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k);

        myOpt_SR4k.optimizePoses(poseChain, frames);

        cout <<"==========================================================" <<endl;
        cout <<"==========================================================" <<endl;
        cout <<"==========================================================" <<endl;

    }



    // PointCloud<pcl::PointXYZ> pc1;
    // PointCloud<pcl::PointXYZ> pc2;
    
    // vector<Point3f> pts1;
    // vector<Point3f> pts2;

    // pts1 = myBase_SR4k.imagToCVpt( f1_4k.depthXYZ, C_sr4k );
    // pts2 = myBase_SR4k.imagToCVpt( f2_4k.depthXYZ, C_sr4k );

    // pc1 = myBase_SR4k.cvPtsToPCL(pts1);
    // pc2 = myBase_SR4k.cvPtsToPCL(pts2);

    // if ( idx == startIdx ){
    //     *pc_all = pc1;
    // }
    // Eigen::Isometry3d T_eigen = myBase_SR4k.cvTtoEigenT(T);
    // pcl::transformPointCloud( *pc_all, *pc_all, T_eigen.matrix() );
    // *pc_all = pc2+*pc_all;
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setLeafSize (0.01f, 0.01f, 0.01f); //1cm
    // sor.setInputCloud (pc_all);
    // sor.filter (*pc_all);
    // pcl::io::savePCDFile( "./"+to_string(idx)+"pc_all.pcd", *pc_all );
    // pcl::io::savePCDFile( "./"+to_string(idx)+".pcd", pc2 );

    // pcl::visualization::CloudViewer viewer( "viewer" );
    // viewer.showCloud( pc_12 );
    // while( !viewer.wasStopped() )
    // {
        
    // }
    for ( int i=0; i<poseChain.size(); i++ )
    {               
        Mat cv44T = (poseChain[i]).T_ab ;
        float roll,pitch,yaw;
        myBase_SR4k.rotMtoRPY(cv44T, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    }
    cout <<"==========================================================" <<endl;
    for ( int i=0; i<frames.size(); i++ )
    {               
        Mat cv44T = (frames[i]).pose ;
        float roll,pitch,yaw;
        myBase_SR4k.rotMtoRPY(cv44T, roll, pitch, yaw);
        cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;
    }

    PointCloud<pcl::PointXYZ> pc;
    vector<Point3f> pt3f = myBase_SR4k.imagToCVpt( (frames[0]).depthXYZ, C_sr4k );
    pc = myBase_SR4k.cvPtsToPCL(pt3f);
    *pc_all = pc;

    for ( int i=1; i<frames.size(); i++ )
    {               
        pt3f = myBase_SR4k.imagToCVpt( (frames[i]).depthXYZ, C_sr4k );
        pc = myBase_SR4k.cvPtsToPCL(pt3f);

        Eigen::Isometry3d T_eigen = myBase_SR4k.cvTtoEigenT( (frames[i]).pose );
        pcl::transformPointCloud( pc, pc, (T_eigen.inverse()).matrix() );
        *pc_all = pc+*pc_all;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setLeafSize (0.01f, 0.01f, 0.01f); //1cm
        sor.setInputCloud (pc_all);
        sor.filter (*pc_all);
    }
    pcl::io::savePCDFile( "./pc_all.pcd", *pc_all );

    return 0;
}


void frame2framePose(int firstF_id,int secondF_id,vector<myPoseAtoB> &poseChain,vector<SR4kFRAME> &frames, 
    pose_estimation myVO_SR4k,slamBase myBase_SR4k ){

    SR4kFRAME f1_4k = frames[ firstF_id-startIdx ];
    SR4kFRAME f2_4k = frames[ secondF_id-startIdx ];

    // pose_estimation myVO_SR4k; 
    // slamBase myBase_SR4k; 
    double overlapRate;
    vector<Point2f> p_UVs1, p_UVs2;
    vector<Point3f> p_XYZs1, p_XYZs2;
    myBase_SR4k.find4kMatches(f1_4k,f2_4k,p_UVs1,p_UVs2,p_XYZs1,p_XYZs2,&overlapRate);

    Mat mat_r, vec_t;
    std::vector<int> inliers;
    cv::Mat T = cv::Mat::eye(4,4,CV_64F);
    myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, inliers, &T );
    

    float roll,pitch,yaw;
    myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
    cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;

    // cout << "T = "<<endl<< T <<endl;

    myPoseAtoB foundPose;
    foundPose.posA_id = firstF_id;
    foundPose.posB_id = secondF_id;
    foundPose.T_ab = T;
    poseChain.push_back(foundPose);

}