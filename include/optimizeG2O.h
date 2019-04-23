// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include <boost/concept_check.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


#include "slamBase.h"

using namespace std;
using namespace cv;


struct myPoseAtoB
{
    int posA_id = 0;
    int posB_id = 0;
    double inlierRatio = 0.0;
    Mat T_ab = cv::Mat::eye(4,4,CV_64F);
    
};

class optimizeG2O{
	public:
		// optimizeG2O();
		// ~optimizeG2O();
		void optimizePoses(vector<myPoseAtoB> poseChain, vector<SR4kFRAME>& frames);
};