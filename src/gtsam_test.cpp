#include <iostream>
#include <ros/ros.h>

// 相关头文件
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;

class GTSAM_Optimizer{
public:
    GTSAM_Optimizer(){
        // 设定第一个节点初值
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise =\
                gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
        gtsam::Pose2 initPose(0, 0, 0);
        graph.addPrior(0, initPose, priorNoise);
        
        gtsam::Pose2 odometry(2, 0, 0);
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =\
                gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.1, 0.3));
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(0,1,odometry,odometryNoise));
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1,2,odometry,odometryNoise));

        initial.insert(0,gtsam::Pose2(0.5,0.0,0.2));
        initial.insert(1,gtsam::Pose2(2.3,0.2,-0.2));
        initial.insert(2,gtsam::Pose2(4.2,-0.1,0.2));
    }

    void update(){
        gtsam::Values res = gtsam::LevenbergMarquardtOptimizer(graph,initial).optimize();
        cout<<"Optimization complete"<<endl;
        cout<<"initial error: "<<graph.error (initial) <<endl;
        cout<<"final error: "<<graph.error (res) <<endl;
        cout<<endl;
        initial.print("initial estimate");
        cout<<endl;
        res.print("final estimate");
    }
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph; 

};


int main(int argc, char** argv){
    ros::init(argc,argv,"gtsam_test_node");
    ros::NodeHandle nh;
    cout<<"hello gtsam demo!"<<endl;
    GTSAM_Optimizer opt;
    opt.update();

    ros::spin();
    return 0;
}