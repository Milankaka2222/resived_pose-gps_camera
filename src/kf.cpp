#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>


using namespace message_filters;
using namespace Eigen;

// 卡尔曼滤波器类
class KalmanFilter {
public:
    KalmanFilter() {
        // 初始化状态量 (假设6维状态量: x, y, z, roll, pitch, yaw)
        x = VectorXd(6);
        x.setZero();
        // 初始化状态协方差矩阵
        P = MatrixXd::Identity(6, 6);
        // 初始化状态转移矩阵
        F = MatrixXd::Identity(6, 6);
        // 初始化观测矩阵
        H = MatrixXd::Identity(6, 6);
        // 初始化过程噪声协方差矩阵
        Q = MatrixXd::Identity(6, 6) * 0.01;
        // 初始化观测噪声协方差矩阵
        R = MatrixXd::Identity(6, 6) * 0.1;
    }

    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    void update(const VectorXd &z) {
        VectorXd y = z - H * x;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
        x = x + K * y;
        P = P - K * H * P;
    }

    VectorXd getState() const { return x; }

private:
    VectorXd x; // 状态量
    MatrixXd P; // 状态协方差矩阵
    MatrixXd F; // 状态转移矩阵
    MatrixXd H; // 观测矩阵
    MatrixXd Q; // 过程噪声协方差矩阵
    MatrixXd R; // 观测噪声协方差矩阵
};

// 卡尔曼滤波器实例
KalmanFilter kf;



// 回调函数
void callback(const geometry_msgs::PoseStamped::ConstPtr& ndt_pose,
              const geometry_msgs::PoseStamped::ConstPtr& gnss_pose,
              ros::Publisher& kf_pose_pub,
              tf::TransformBroadcaster& tf_broadcaster)
{
    // 将PoseStamped转换为Eigen::Affine3d
    //std::cout << "Received NDT pose: " << ndt_pose->pose.position.x << ", " << ndt_pose->pose.position.y << ", " << ndt_pose->pose.position.z << std::endl;
    tf::Transform ndt_pose_tf, gnss_pose_tf;
    tf::poseMsgToTF(ndt_pose->pose, ndt_pose_tf);
    tf::poseMsgToTF(gnss_pose->pose, gnss_pose_tf);
    Eigen::Affine3d ndt_pose_eigen, gnss_pose_eigen, transform;
    tf::transformTFToEigen(ndt_pose_tf, ndt_pose_eigen);
    tf::transformTFToEigen(gnss_pose_tf, gnss_pose_eigen);
    //transform = ndt_pose_eigen.inverse() * camera_pose_eigen;
    transform = gnss_pose_eigen.inverse() * ndt_pose_eigen;
    std::cout << "transform: " << transform.matrix() << std::endl;

    // 将变换矩阵转换为观测值
    VectorXd z(6);
    std::cout<<"z:"<<z<<std::endl;
    Vector3d translation = transform.translation();
    Quaterniond rotation(transform.rotation());

    // 必须提供正确的顺序： yaw(2), pitch(1), roll(0)
    Vector3d euler_angles = rotation.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
    z << translation.x(), translation.y(), translation.z(),
        euler_angles.x(), euler_angles.y(), euler_angles.z();

    // 预测和更新
    kf.predict();
    kf.update(z);

    // 获取卡尔曼滤波器修正后的状态
    VectorXd state = kf.getState();
    
    ROS_INFO("Estimated state:");
    ROS_INFO_STREAM(state);

   // 将修正后的状态转换为Eigen::Affine3d
    Eigen::Affine3d corrected_transform;
    corrected_transform.setIdentity();

    // 设置修正后的平移部分
    corrected_transform.translation() << state(0), state(1), state(2);

    // 设置修正后的旋转部分（使用ZYX欧拉角顺序）
    Eigen::AngleAxisd roll_aax(state(3), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_aax(state(4), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_aax(state(5), Eigen::Vector3d::UnitZ());

    corrected_transform.rotate(yaw_aax * pitch_aax * roll_aax);

    // 将修正变换应用于原始的GNSS姿态
    Eigen::Affine3d corrected_gnss_pose = gnss_pose_eigen * corrected_transform;

    // 将修正后的GNSS姿态转换为PoseStamped并发布
    geometry_msgs::PoseStamped corrected_gnss_pose_msg;
    corrected_gnss_pose_msg.header.stamp = ndt_pose->header.stamp;
    corrected_gnss_pose_msg.header.frame_id = "map"; // 根据你的需求设置坐标系

    // 设置位置
    corrected_gnss_pose_msg.pose.position.x = corrected_gnss_pose.translation().x();
    corrected_gnss_pose_msg.pose.position.y = corrected_gnss_pose.translation().y();
    corrected_gnss_pose_msg.pose.position.z = corrected_gnss_pose.translation().z();

    // 设置姿态
    Eigen::Quaterniond corrected_orientation(corrected_gnss_pose.rotation());
    corrected_gnss_pose_msg.pose.orientation.x = gnss_pose->pose.orientation.x;
    corrected_gnss_pose_msg.pose.orientation.y = gnss_pose->pose.orientation.y;
    corrected_gnss_pose_msg.pose.orientation.z = gnss_pose->pose.orientation.z;
    corrected_gnss_pose_msg.pose.orientation.w = gnss_pose->pose.orientation.w;

    // 发布修正后的GNSS姿态
    kf_pose_pub.publish(corrected_gnss_pose_msg);

    tf::StampedTransform received_transform;
    received_transform.stamp_ =ros::Time::now();
    received_transform.frame_id_ = "map";  // 假设 received_pose 在 map 坐标系下
    received_transform.child_frame_id_ = "received_pose";
    received_transform.setOrigin(tf::Vector3(corrected_gnss_pose.translation().x(), corrected_gnss_pose.translation().y(), corrected_gnss_pose.translation().z()));
    received_transform.setRotation(tf::Quaternion(gnss_pose->pose.orientation.x, gnss_pose->pose.orientation.y, gnss_pose->pose.orientation.z, gnss_pose->pose.orientation.w));
    tf_broadcaster.sendTransform(received_transform);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_pose_converter");
    ros::NodeHandle nh;

    // 创建订阅者
    message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pose_sub(nh, "ndt_pose", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> gnss_pose_sub(nh, "gnss_pose", 10);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;

    // 创建同步器
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),ndt_pose_sub, gnss_pose_sub);

    // 创建位姿发布者
    ros::Publisher kf_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("kf_pose", 1);

    tf::TransformBroadcaster tf_broadcaster;
    sync.registerCallback(boost::bind(&callback, _1, _2, boost::ref(kf_pose_pub),boost::ref(tf_broadcaster)));

    ros::spin();
    return 0;
}
