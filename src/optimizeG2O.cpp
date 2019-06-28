#include "optimizeG2O.h"

int optimizeG2O::trackFromRoot(vector<myPoseAtoB> poseChain, int frameID){
    int ReachEnd =0;
    for ( int k=0; k<poseChain.size(); k++ )
    {
        if(poseChain[k].posB_id==frameID){
            ReachEnd = 1;
        }
    }
    // todo:
    // check if all frame ID can track back to first id

    return ReachEnd;
}

void optimizeG2O::optimizePoses(vector<myPoseAtoB> poseChain, vector<SR4kFRAME>& frames){

    for ( int i=0; i<frames.size(); i++ )
    {
        if (trackFromRoot(poseChain, (frames[i]).frameID)>0){
            frames[i].valid =1;
        }else{
            cout << "separated vo!!!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
            cout << "break at frame: "<< frames[i].frameID <<endl;
            cout << "separated vo!!!!!!!!!!!!!!!!!!!!!!!!!" <<endl<<endl;
        }
    }



    g2o::SparseOptimizer optimizer;
    
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 


    std::unique_ptr<SlamLinearSolver> linearSolver ( new SlamLinearSolver());
    linearSolver->setBlockOrdering( false );

    std::unique_ptr<SlamBlockSolver> blockSolver ( new SlamBlockSolver ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(blockSolver) );


    optimizer.setAlgorithm( solver );
    optimizer.setVerbose( false );

    // vertex
    for ( int i=0; i<frames.size(); i++ )
    {
        // g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); 

        if (frames[i].valid >0)
        {
            v->setEstimate( g2o::SE3Quat() );
            v->setId( (frames[i]).frameID );
            optimizer.addVertex(v);
        }
    }

    // cout <<"i am here"<<endl;

    // edges
    for ( int i=0; i<poseChain.size(); i++ )
    {	        	
	    Eigen::Isometry3d T_eigen = Eigen::Isometry3d::Identity();
	    Mat cv44T = (poseChain[i]).T_ab ;

	    Eigen::Matrix3d r3v;

	    Mat mat_r = cv44T(cv::Rect(0,0,3,3));
	    cv::cv2eigen(mat_r, r3v);
	    Eigen::AngleAxisd angle(r3v);
	    T_eigen = angle;
	    T_eigen(0,3) = cv44T.at<double>(0,3); 
	    T_eigen(1,3) = cv44T.at<double>(1,3); 
	    T_eigen(2,3) = cv44T.at<double>(2,3);

	    g2o::EdgeSE3* edge_T = new g2o::EdgeSE3();

	    edge_T->vertices() [0] = optimizer.vertex( (poseChain[i]).posA_id );
	    edge_T->vertices() [1] = optimizer.vertex( (poseChain[i]).posB_id );

	    // Eigen::Matrix<double, 6, 6> information_T = 10000000*((poseChain[i]).inlierRatio)*Eigen::Matrix< double, 6,6 >::Identity();
	    Eigen::Matrix<double, 6, 6> information_T = 10000000*Eigen::Matrix< double, 6,6 >::Identity();
	    // information_T(0,0) = 1000000;
	    // information_T(1,1) = 1000000;
	    // information_T(2,2) = 1000000;

	    // information_T(3,3) = 1000000;
	    // information_T(4,4) = 1000000;
	    // information_T(5,5) = 1000000;

	    edge_T->setInformation( information_T );
	    edge_T->setMeasurement( T_eigen );
	    optimizer.addEdge(edge_T);
	}
	optimizer.save("./result_before.g2o");
	optimizer.initializeOptimization();
    optimizer.optimize(1000);
    optimizer.save("./result_after.g2o");

    // update all poses

    // cout << "frames.size() "<< frames.size()<<endl;
    for ( int i=1; i<frames.size(); i++ )
    {
        if (frames[i].valid >0){
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>( optimizer.vertex( (frames[i]).frameID ) );
            Eigen::Isometry3d pose = v->estimate();
            Mat cvT;
            eigen2cv(pose.matrix(),cvT);
            (frames[i]).pose = cvT;
        }
    }

    // cout <<"i am out"<<endl;
}