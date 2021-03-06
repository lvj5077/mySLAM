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

const int startIdx=1;
const int endIdx=584; //584

void frame2framePose(int firstF_id,int secondF_id,vector<myPoseAtoB> &poseChain,vector<SR4kFRAME> &frames, 
    pose_estimation myVO_SR4k,slamBase myBase_SR4k, int loopclosure );

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
    C_sr4k.exp = 32;//8,32;


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
    // string data_path = "/Users/lingqiujin/Data/trajectory_exp7/d1_";
    string data_path = "/Users/lingqiujin/Data/RV_Data2/d1_";

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
        
        frame2framePose(firstF_id,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,0);

        // srand (time(NULL));
        // int randomCheckID = rand()%frames.size()+ startIdx; 
        // cout << randomCheckID<<" " << secondF_id << endl;
        // frame2framePose(randomCheckID,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);


        // if((secondF_id-startIdx)>7){
        //     frame2framePose(secondF_id-2,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-3,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-4,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-6,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     // frame2framePose(secondF_id-5,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        // }
        // if((secondF_id-startIdx)>7){
        //     frame2framePose(secondF_id-6,secondF_id-3,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-5,secondF_id-3,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-4,secondF_id-3,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-2,secondF_id-3,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        //     frame2framePose(secondF_id-1,secondF_id-3,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        // }

        // myOpt_SR4k.optimizePoses(poseChain, frames);

        cout <<"==========================================================" <<endl;
        cout <<"==========================================================" <<endl;
        cout <<"==========================================================" <<endl;

    }

    myOpt_SR4k.optimizePoses(poseChain, frames);

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
    // for ( int i=0; i<poseChain.size(); i++ )
    // {               
    //     cout << poseChain[i].posA_id<<"   " << poseChain[i].posB_id<<endl;
    // }
    cout <<"==========================================================" <<endl;
    ofstream myfile;
    myfile.open ("/Users/lingqiujin/work/mySLAM/data.txt");
    for ( int i=0; i<frames.size(); i++ )
    {   
        if (frames[i].valid <1){
            cout <<"No pose_estimation for frame: "<<frames[i].frameID<<endl;
        }
        else{
            Mat cv44T = (frames[i]).pose ;
            float roll,pitch,yaw;
            myBase_SR4k.rotMtoRPY(cv44T, roll, pitch, yaw);
            // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;

            stringstream ss;
            ss << roll<<","<<pitch<<","<<yaw<<","<<cv44T.at<double>(0,3) << ","<< cv44T.at<double>(1,3)<< "," << cv44T.at<double>(2,3);
            string rpyxyz = ss.str();
            myfile << rpyxyz <<"\n";
        }
    }
    myfile.close();

    for (int idx=startIdx;idx<endIdx;idx++){
        int firstF_id = idx;
        int secondF_id = idx+1;
        // srand (time(NULL));
        // int randomCheckID = rand()%frames.size()+ startIdx; 
        // frame2framePose(randomCheckID,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        if((secondF_id-startIdx)>8){
            frame2framePose(secondF_id-2,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            frame2framePose(secondF_id-3,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            frame2framePose(secondF_id-4,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            frame2framePose(secondF_id-5,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            frame2framePose(secondF_id-6,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            // frame2framePose(secondF_id-7,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            // frame2framePose(secondF_id-9,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            // frame2framePose(secondF_id-12,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
            // frame2framePose(secondF_id-5,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,1);
        }
    }
    myOpt_SR4k.optimizePoses(poseChain, frames);

    myfile.open ("/Users/lingqiujin/work/mySLAM/dataOpt.txt");
    for ( int i=0; i<frames.size(); i++ )
    {   
        if (frames[i].valid <1){
            cout <<"No pose_estimation for frame: "<<frames[i].frameID<<endl;
        }
        else{
            Mat cv44T = (frames[i]).pose ;
            float roll,pitch,yaw;
            myBase_SR4k.rotMtoRPY(cv44T, roll, pitch, yaw);
            // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;

            stringstream ss;
            ss << roll<<","<<pitch<<","<<yaw<<","<<cv44T.at<double>(0,3) << ","<< cv44T.at<double>(1,3)<< "," << cv44T.at<double>(2,3);
            string rpyxyz = ss.str();
            myfile << rpyxyz <<"\n";
        }
    }
    myfile.close();
    return 0;
}


void frame2framePose(int firstF_id,int secondF_id,vector<myPoseAtoB> &poseChain,vector<SR4kFRAME> &frames, 
    pose_estimation myVO_SR4k,slamBase myBase_SR4k, int loopclosure){

    int posechainUpdated = 0;
    int inliers_threshold = 10;
    double inlierR_threshold = 0.6;
    if (loopclosure >0){
        inliers_threshold = 10;
        inlierR_threshold = 0.8;
    }

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
    if (p_XYZs1.size()<inliers_threshold){
        cout << "not enough inputs!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    }
    else{
        myVO_SR4k.RANSACpose3d3d_SVD(p_XYZs2, p_XYZs1, mat_r, vec_t, inliers, &T );

        double inlierR = (double)inliers.size() /p_XYZs2.size() ;
        cout << "inlier ratio "<< inlierR <<endl;
        if((inliers.size()<inliers_threshold && inlierR<inlierR_threshold)){
            cout << "bad pose_estimation!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        }
        else    // update pose chain
        {
            myPoseAtoB foundPose;
            foundPose.posA_id = firstF_id;
            foundPose.posB_id = secondF_id;
            foundPose.T_ab = T;
            foundPose.inlierRatio = inlierR;
            poseChain.push_back(foundPose);
            posechainUpdated = 1;
        }
    }

    if (loopclosure<1 && posechainUpdated<1){
        firstF_id = firstF_id-1;
        if (firstF_id<startIdx || (secondF_id-firstF_id)>5){
            cout << "VO failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            cout << "VO failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            cout << "VO failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            cout << "VO failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            cout << "VO failed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        }
        else{
          frame2framePose(firstF_id,secondF_id,poseChain,frames,myVO_SR4k,myBase_SR4k,0); 
        }
       
    }
    
    
    // float roll,pitch,yaw;
    // myBase_SR4k.rotMtoRPY(T, roll, pitch, yaw);
    // cout << "roll " << roll<<" pitch " << pitch<<" yaw " << yaw<<endl;

    // cout << "T = "<<endl<< T <<endl;


}