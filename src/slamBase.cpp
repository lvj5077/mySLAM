#include "slamBase.h"
#include <opencv2/ml.hpp>
// #include <miniflann.hpp>

#include <opencv2/opencv.hpp>  
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp> // SIFT

slamBase::slamBase(void)
{
	// C.cx = 325.141442;
	// C.cy = 249.701764;
	// C.fx = 520.908620;
	// C.fy = 521.007327;
	// C.scale = 5208;

	C.cx = 325.1;
	C.cy = 249.7;
	C.fx = 520.9;
	C.fy = 521.0;
	C.scale = 5000;

    C.height = 480;
    C.width = 640;

	C.depthL = 0.18;
	C.depthH = 5.0;

    C.exp = 50;
    // cout << "Object is being created" << endl;
}

slamBase::~slamBase(void)
{
    // cout << "Object is being deleted" << endl;
}


void slamBase::setCamera(CAMERA_INTRINSIC_PARAMETERS inC){
	C = inC;
}

CAMERA_INTRINSIC_PARAMETERS slamBase::getCamera(){
	return C;
}


SR4kFRAME slamBase::readSRFrame( string inFileName){

    cout << "load "<<inFileName<<endl;


    SR4kFRAME f;

    int width = 176;
    int height = 144;

    cv::Mat I_gray = cv::Mat::zeros(height,width,CV_64F);
    cv::Mat I_z = cv::Mat::zeros(height,width,CV_64F);

    int size[3] = { width, height, 3 };
    cv::Mat I_depth(3, size, CV_64F, cv::Scalar(10));

    ifstream inFile(inFileName);
    string str;

    int lineIdx = 0;

    double temp = 0;
    while (getline(inFile, str))
    {
        lineIdx ++;
        if (lineIdx > 0 && lineIdx<(height+1))
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_depth.at<double>(int(lineIdx-1),i,2) ;
                I_z.at<double>(int(lineIdx-1),i) = I_depth.at<double>(int(lineIdx-1),i,2);
            }
        }

        if (lineIdx > (height+1)*1 && lineIdx<(height+1)*2)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> temp ;
                I_depth.at<double>(int(lineIdx-1 -(height+1)*1 ),i,0) = -1* temp ;
            }
        }
        if (lineIdx > (height+1)*2 && lineIdx<(height+1)*3)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> temp;
                I_depth.at<double>(int(lineIdx-1 -(height+1)*2 ),i,1) = -1* temp;
            }
        }

        if (lineIdx > (height+1)*3 && lineIdx<(height+1)*4)
        {
            for (int i = 0; i < width; i++)
            {
                inFile >> I_gray.at<double>(int(lineIdx-1 -(height+1)*3 ),i,0) ;
            }
        }

    }

    I_gray = I_gray/C.exp ; //12bit
    I_gray.convertTo(I_gray,CV_8U);

    // I_gray.convertTo(I_gray, CV_8U, 1.0 / 256, 0);

    // equalizeHist( I_gray, I_gray );

    // GaussianBlur(I_gray, I_gray, Size(3, 3), 1);

    // GaussianBlur(I_z, I_z, Size(3, 3), 1);

    f.rgb = I_gray.clone();
    f.depthXYZ = I_depth.clone();


    I_z = I_z*1000; 
    I_z.convertTo(I_z,CV_16U);
    f.z= I_z.clone();

    // for (int i =0;i<5;i++){
    //     cout << I_z.at<unsigned short>(2,i) <<"  "<<I_depth.at<double>(2,i,2) <<endl;
    // }

    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create(0,5,.002);
    f2d->detect ( f.rgb,f.kp );
    f2d->compute ( f.rgb, f.kp, f.desp );

    return f;
}


void slamBase::find4kMatches(SR4kFRAME & frame1, SR4kFRAME & frame2, 
            vector<Point2f> &p_UVs1,vector<Point2f> &p_UVs2,vector<Point3f> &p_XYZs1,vector<Point3f> &p_XYZs2, double* overlapRate){

    p_UVs1.clear();
    p_UVs2.clear();
    p_XYZs1.clear();
    p_XYZs2.clear();

    vector<Point2f> tp_UVs1;
    vector<Point2f> tp_UVs2;

    // vector< DMatch > matches;
    // BFMatcher matcher(NORM_L2);
    // matcher.match(descriptors_1,descriptors_2, matches);
    // nth_element(matches.begin(),matches.begin()+50,matches.end());
    // matches.erase(matches.begin()+50,matches.end());

    vector< DMatch > goodMatches;

    cout << "found " << (frame1.desp).rows<<endl;
    int k = 2; 
    double sum_dis = 0;     
    double dis_ratio = 0.5; 

    cv::flann::Index* mpFlannIndex = new cv::flann::Index((frame1.desp), cv::flann::KDTreeIndexParams()); 

    int num_features = (frame2.desp).rows; 
    cv::Mat indices(num_features, k, CV_32S); 
    cv::Mat dists(num_features, k, CV_32F); 
    cv::Mat relevantDescriptors = (frame2.desp).clone(); 

    mpFlannIndex->knnSearch(relevantDescriptors, indices, dists, k, flann::SearchParams(16) ); 

    int* indices_ptr = indices.ptr<int>(0); 
    float* dists_ptr = dists.ptr<float>(0); 
    cv::DMatch m;
    set<int> train_ids; 
    for(int i=0; i<indices.rows; i++){
        float dis_factor = dists_ptr[i*2] / dists_ptr[i*2+1]; 
        if(dis_factor < dis_ratio ){
            int train_id = indices_ptr[i*2]; 
            if(train_ids.count(train_id) > 0) { // already add this feature 
                // TODO: select the best matched pair 
                continue; 
            }
            // add this match pair  
            m.trainIdx = train_id; 
            m.queryIdx = i; 
            m.distance = dis_factor;
            goodMatches.push_back(m);
            train_ids.insert(train_id); 
        }
    }
    // goodMatches = matches;
    cv::Mat imgMatches;

    // cout <<"Find total "<<goodMatches.size()<<" matches."<<endl;
    // cv::drawMatches( rgb2, keypoints_2, rgb1, keypoints_1, goodMatches, imgMatches );
    // cv::imshow( "Good matches", imgMatches );
    // cv::waitKey( 0 );

    vector< DMatch > valid3Dmatches;
    int dupx1,dupy1,dupx2,dupy2;
    for ( DMatch m:goodMatches )
    {

        cv::Point2f p1 = (frame1.kp)[m.trainIdx].pt;
        cv::Point2f p2 = (frame2.kp)[m.queryIdx].pt;
        double d1 = (frame1.depthXYZ).at<double>(int(p1.y),int(p1.x),2);
        double d2 = (frame2.depthXYZ).at<double>(int(p2.y),int(p2.x),2);

        // if ( d1<C.depthL||d1>C.depthH || d2<C.depthL||d2>C.depthH)   // bad depth
        if ( d1 == 0 || d2 == 0 || d1>9 || d2>9) 
            continue;

        if (dupx1 == int(p1.x) && dupy1 == int(p1.y) && dupx2 == int(p2.x) && dupy2 == int(p2.y))
            continue;

        dupx1 = int(p1.x);
        dupy1 = int(p1.y);
        dupx2 = int(p2.x);
        dupy2 = int(p2.y);

        tp_UVs1.push_back( p1 );
        tp_UVs2.push_back( p2 );

        valid3Dmatches.push_back(m);

    }

    p_XYZs1.clear();
    p_XYZs2.clear();


    vector< DMatch > distMatches;
    for ( int i=0;i<tp_UVs1.size();i++)
    {

        cv::Point3f p_XYZ1;
        p_XYZ1.x = (frame1.depthXYZ).at<double>(int(tp_UVs1[ i ].y),int(tp_UVs1[ i ].x),0);
        p_XYZ1.y = (frame1.depthXYZ).at<double>(int(tp_UVs1[ i ].y),int(tp_UVs1[ i ].x),1);
        p_XYZ1.z = (frame1.depthXYZ).at<double>(int(tp_UVs1[ i ].y),int(tp_UVs1[ i ].x),2);

        cv::Point3f p_XYZ2;
        p_XYZ2.x = (frame2.depthXYZ).at<double>(int(tp_UVs2[ i ].y),int(tp_UVs2[ i ].x),0);
        p_XYZ2.y = (frame2.depthXYZ).at<double>(int(tp_UVs2[ i ].y),int(tp_UVs2[ i ].x),1);
        p_XYZ2.z = (frame2.depthXYZ).at<double>(int(tp_UVs2[ i ].y),int(tp_UVs2[ i ].x),2);

        // cout << norm(tp_UVs1[i]-tp_UVs2[i]) << "  " << norm(tp_XYZs1[i]-tp_XYZs2[i])<<endl;
        // if ( norm(tp_UVs1[i]-tp_UVs2[i])> 40 && norm(tp_XYZs1[i]-tp_XYZs2[i])>1)
        if ( norm(p_XYZ1-p_XYZ2)>1)
            continue;

        p_UVs1.push_back( tp_UVs1[ i ] );
        p_UVs2.push_back( tp_UVs2[ i ] );

        p_XYZs1.push_back( p_XYZ1 );
        p_XYZs2.push_back( p_XYZ2 );

        distMatches.push_back(  valid3Dmatches[ i ]   );
    }

    cout << "p_XYZs1.size() "<<p_XYZs1.size()  << endl;

    * overlapRate = (double) distMatches.size()/indices.rows ; 
    cout << "similarity: " <<  * overlapRate <<" "<<  distMatches.size() <<" "<< indices.rows <<endl;
    

    // cout<<"3d-3d tp_XYZs2: "<<tp_XYZs2<< "   "<< tp_XYZs2 << endl;
    // cout <<"Find total "<<distMatches.size()<<" matches."<<endl;
    // cv::drawMatches( (frame2.rgb), (frame2.kp), (frame1.rgb), (frame1.kp),  distMatches, imgMatches );
    // cv::imshow( "valid3Dmatches", imgMatches );
    // cv::waitKey( 0 );

    // cout<<"3d-3d pairs: "<<p_XYZs1.size() <<endl;
    // cout<<"3d-3d pairs: "<<p_XYZs1<< "   "<< p_XYZs2 << endl;
}



cv::Point3f slamBase::point2dTo3d( cv::Point2f& point, double& d, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p;
    p.z = float( d) ;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;

    return p;
}

double slamBase::reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & mat_r, Mat & vec_t ){
	double rpE = 0;

    cv::Mat T = cv::Mat::eye(4,4,CV_64F);

    mat_r.copyTo(T(cv::Rect(0, 0, 3, 3)));
    vec_t.copyTo(T(cv::Rect(3, 0, 1, 3)));
	

	rpE = reprojectionError(p_XYZs1,p_XYZs2,T);

 //    for (int i=0;i<p_XYZs1.size();i++){

 //        rpE = rpE + norm(mat_r* (Mat_<double>(3,1)<<p_XYZs1[1].x, p_XYZs1[1].y, p_XYZs1[1].z) + vec_t - 
	// 					(Mat_<double>(3,1)<<p_XYZs2[1].x, p_XYZs2[1].y, p_XYZs2[1].z) ) ;
 //    }

 //    rpE = rpE/p_XYZs1.size();

    // for (int i=0;i<5;i++){
    //     cv::Point3f pd1 = p_XYZs1[i];
    //     cv::Point3f pd2 = p_XYZs2[i];
    //     cout << "pd2 "<<(p_XYZs2[i])<<endl;
    //     cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
    //     cv::Mat dstMat = T*ptMat;
    //     cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
    //     cout << "projPd1 "<<projPd1<<endl;
    //     cout << ( mat_r* (Mat_<double>(3,1)<<p_XYZs1[i].x, p_XYZs1[i].y, p_XYZs1[i].z) + vec_t ).t()<<endl;
    // }
	return rpE;
}

double slamBase::reprojectionError( vector<Point3f> & p_XYZs1, vector<Point3f> & p_XYZs2, Mat & T ){
	// cout << "T ==================================== "<<endl<<T<<endl;
	double rpE = 0;
        for (int i=0;i<p_XYZs1.size();i++){
            cv::Point3f pd1 = p_XYZs1[i];
            cv::Point3f pd2 = p_XYZs2[i];

            cv::Mat ptMat = (cv::Mat_<double>(4, 1) << pd1.x, pd1.y, pd1.z, 1);
            cv::Mat dstMat = T*ptMat;
            cv::Point3f projPd1(dstMat.at<double>(0,0), dstMat.at<double>(1,0),dstMat.at<double>(2,0));
            // cout <<pd1<<" "<< pd2<< "  "<<projPd1<< "  "<< norm(projPd1-pd2)<< endl;

            rpE = rpE + norm(projPd1-pd2);
            // if (norm(projPd1-pd2)>0.3){
            //     cout <<"bad point warnning"<<endl;
            // }
        }
        rpE = rpE/p_XYZs1.size();
	return rpE;
}

void slamBase::rotMtoRPY(Mat &mat_r, float &roll,float &pitch,float &yaw){
    float sy= sqrt(mat_r.at<double>(0,0) * mat_r.at<double>(0,0) +  mat_r.at<double>(1,0) * mat_r.at<double>(1,0) );
    roll = 180/3.14159265*atan2(mat_r.at<double>(2,1) , mat_r.at<double>(2,2));
    pitch = 180/3.14159265*atan2(-mat_r.at<double>(2,0), sy);
    yaw = 180/3.14159265*atan2(mat_r.at<double>(1,0), mat_r.at<double>(0,0));
    // if (pitch<0){
    //     pitch = -pitch;
    //     roll = -roll;
    //     yaw = -yaw;
    // }
}



// Calculates rotation matrix given euler angles.
Mat slamBase::eulerAnglesToRotationMatrix(float roll,float pitch,float yaw)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(roll),   -sin(roll),
               0,       sin(roll),   cos(roll)
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(pitch),    0,      sin(pitch),
               0,               1,      0,
               -sin(pitch),   0,      cos(pitch)
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(yaw),    -sin(yaw),      0,
               sin(yaw),    cos(yaw),       0,
               0,               0,          1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
     
    return R;
 
}

pcl::PointCloud<pcl::PointXYZ> slamBase::cvPtsToPCL(vector<Point3f> &p_XYZs)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize (p_XYZs.size());
    for (size_t i=0; i<p_XYZs.size(); i++) {
        cloud.points[i].x = p_XYZs[i].x;
        cloud.points[i].y = p_XYZs[i].y;
        cloud.points[i].z = p_XYZs[i].z;
    }
    cloud.height = 1;
    cloud.width = cloud.points.size();
    return cloud;
}

vector<Point3f> slamBase::imagToCVpt( Mat depth, CAMERA_INTRINSIC_PARAMETERS& camera ){
    vector<Point3f> pts_cv;

    for(int i=0;i<camera.width;i++){
        for(int j=0;j<camera.height;j++){
            cv::Point3f p;
            double d = depth.at<double>(j,i,2);
            p.z = float( d) ;
            p.x = ( i - camera.cx) * p.z / camera.fx;
            p.y = ( j - camera.cy) * p.z / camera.fy;


            pts_cv.push_back(p);
        }
    }

    return pts_cv;
}

Eigen::Isometry3d  slamBase::cvTtoEigenT( Mat cv44T){
    Eigen::Isometry3d T_eigen = Eigen::Isometry3d::Identity();

    Eigen::Matrix3d r3v;

    Mat mat_r = cv44T(cv::Rect(0,0,3,3));
    cv::cv2eigen(mat_r, r3v);
    Eigen::AngleAxisd angle(r3v);
    T_eigen = angle;
    T_eigen(0,3) = cv44T.at<double>(0,3); 
    T_eigen(1,3) = cv44T.at<double>(1,3); 
    T_eigen(2,3) = cv44T.at<double>(2,3);

    return T_eigen;
}
