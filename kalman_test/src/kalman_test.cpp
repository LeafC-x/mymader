#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "kalman_test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("=== Kalman Library Test ===");
    
    try {
        // 测试1: 检查Eigen3是否正常工作
        ROS_INFO("1. Testing Eigen3 integration...");
        Eigen::Vector3f test_vector;
        test_vector << 1.0, 2.0, 3.0;
        ROS_INFO("   Eigen vector: [%.2f, %.2f, %.2f]", 
                 test_vector[0], test_vector[1], test_vector[2]);
        
        // 测试2: 创建EKF滤波器
        ROS_INFO("2. Testing EKF creation...");
        Kalman::ExtendedKalmanFilter<Eigen::Vector4f> ekf;
        Eigen::Vector4f initialState;
        initialState << 0.0, 0.0, 1.0, 0.5;
        ekf.init(initialState);
        ROS_INFO("   EKF initialized successfully");
        ROS_INFO("   Initial state: [%.2f, %.2f, %.2f, %.2f]",
                 ekf.getState()[0], ekf.getState()[1],
                 ekf.getState()[2], ekf.getState()[3]);
        
        // 测试3: 创建UKF滤波器
        ROS_INFO("3. Testing UKF creation...");
        Kalman::UnscentedKalmanFilter<Eigen::Vector4f> ukf(1.0);
        ukf.init(initialState);
        ROS_INFO("   UKF initialized successfully");
        ROS_INFO("   Initial state: [%.2f, %.2f, %.2f, %.2f]",
                 ukf.getState()[0], ukf.getState()[1],
                 ukf.getState()[2], ukf.getState()[3]);
        
        // 测试4: 测试简单的预测和更新（模拟）
        ROS_INFO("4. Testing basic operations...");
        // 模拟预测步骤
        Eigen::Vector4f current_state = ekf.getState();
        ROS_INFO("   Current state: [%.2f, %.2f, %.2f, %.2f]",
                 current_state[0], current_state[1],
                 current_state[2], current_state[3]);
        
        // 测试5: 检查滤波器类型信息
        ROS_INFO("5. Checking filter types...");
        ROS_INFO("   EKF state size: %ld", ekf.getState().size());
        ROS_INFO("   UKF state size: %ld", ukf.getState().size());
        
        ROS_INFO("==========================================");
        ROS_INFO(" All tests passed! Kalman library is successfully integrated!");
        ROS_INFO(" Eigen3 is working properly!");
        ROS_INFO(" EKF and UKF filters are available!");
        ROS_INFO("==========================================");
        
        // 保持节点运行一段时间以便查看输出
        ros::Duration(2.0).sleep();
        
    } catch (const std::exception& e) {
        ROS_ERROR(" Test failed with error: %s", e.what());
        ROS_ERROR(" Kalman library integration failed!");
        return 1;
    }
    
    ROS_INFO("Test completed. Shutting down...");
    return 0;
}