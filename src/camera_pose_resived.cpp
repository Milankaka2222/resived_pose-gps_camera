#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include<sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#define  min_ 0xc0c0c0c0
#define LAT_START 22.761031217740276
#define LONG_START 114.41159539483721
#define ALT_START 30.975505828857422
#define LSB_M_TO_LAT_LONG 8.993216059e-6
const float DISTANCE_THRESHOLD = 2.0; 

// 用于存储变换矩阵和坐标点
struct Transform {
    std::array<float,9> rotation; // 3x3旋转矩阵
    std::vector<float> translation{3, 0.0f}; // 平移向量
    std::vector<float> position{3, 0.0f}; // 坐标点
};


#define FLITTER_LENGTH 10 //定义滑动滤波窗口的长度,长度越大,精度高,但延迟越大
unsigned int flitter_length_cnt_x = 0, flitter_length_cnt_y = 0, flitter_length_cnt_z = 0;//定义一个滤波计数器
double flitter_buf_x[FLITTER_LENGTH];//定义数组,作为滑动窗口滤波的队列
double flitter_buf_y[FLITTER_LENGTH];//定义数组,作为滑动窗口滤波的队列
double flitter_buf_z[FLITTER_LENGTH];//定义数组,作为滑动窗口滤波的队列

double SlidingWindow_Flitter(unsigned int flitter_length,unsigned int *flitter_cnt,double buf[],double input)
{
    unsigned int cnt;
    double temp = 0,output = 0;
    buf[*flitter_cnt] = input;
    *flitter_cnt += 1;
    if(*flitter_cnt >= flitter_length) *flitter_cnt = 0;
    for(cnt = 0;cnt < flitter_length;cnt++)
    {
        temp += buf[cnt];
        // ROS_INFO("%d   %f   %f",cnt, buf[cnt], temp);
    }

    output = temp / flitter_length;
    return output;
}
// 计算两点之间的欧氏距离
float distance(const std::vector<float>& p1, const std::vector<float>& p2) {
    float sum = 0.0f;
    for (size_t i = 0; i < p1.size(); ++i) {
        sum += std::pow(p1[i] - p2[i], 2);
    }
    return std::sqrt(sum);
}

class PoseTransformer {
public:
    PoseTransformer() {
        std::string filename = "/home/roam/work/posdsfe/convert.txt";
        // 订阅Camera_Pose话题
        pose_sub = nh.subscribe("Camera_Pose", 1, &PoseTransformer::poseCallback, this);
        cam_pose_sub = nh.subscribe("/gnss_pose", 1, &PoseTransformer::camposeCallback, this);
        status_sub = nh.subscribe("/gps/fix", 1, &PoseTransformer::statusCallback, this);
        // 发布/received_pose话题
        received_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("resived_pose", 1);
        lat_long_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("lat_long_pose", 1);
        timer = nh.createTimer(ros::Duration(2.0), &PoseTransformer::timerCallback, this);
        ros::NodeHandle nh("~");
        nh.getParam("pose_path", filename);
        std::cout << "****filename********"<< filename <<std::endl;
        // 初始化变换矩阵和坐标点的向量

        
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_FATAL("无法打开文件");
            ros::shutdown();
        }

        std::string line;
        int line_num = 0;
        while (std::getline(file, line)) {
            ++line_num;
            std::stringstream ss(line);
            Transform transform;
            float value;

            for (int i = 0; i < 3; ++i) {
                ss >> transform.position[i];
            }

            //读取旋转矩阵
            for (int i = 0; i < 9; ++i) {
                ss >> transform.rotation[i];
            }

            //读取平移向量
            for (int i = 0; i < 3; ++i) {
                ss >> transform.translation[i];
            }

        

            transforms.push_back(transform);
            transforms[line_num-1].position[2] = transform.position[2];
            transforms[line_num-1].translation[2] = transform.translation[2];
            
            std::cout << "Transform (Line " << line_num << "): ";
            for (int i = 0; i < 3; ++i) {
                std::cout << transform.position[i] << " ";
            }
            for (int i = 0; i < 9; ++i) {
                std::cout << transform.rotation[i] << " ";
            }
            for (int i = 0; i < 3; ++i) {
                std::cout << transform.translation[i] << " ";
            }
            std::cout << std::endl;
        }
        file.close();
    }
  

private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Publisher received_pose_pub;
    ros::Publisher lat_long_pose_pub;
    ros::Subscriber cam_pose_sub;
    ros::Subscriber status_sub;
    ros::Timer timer;
    ros::Time last_received_time_;

    std::vector<Transform> transforms;
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;

sensor_msgs::NavSatFix status_msg;
    void statusCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        status_msg = *msg;

    }
    
geometry_msgs::PoseStamped cam_pose_msg;
    void camposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) { 
        cam_pose_msg = *msg; 
    }


    void timerCallback(const ros::TimerEvent& event) {
        // Check if the last received message is too old
        ros::Duration time_since_last_message = ros::Time::now() - last_received_time_;
        if (time_since_last_message > ros::Duration(1.0)) { // If no message for more than 1 second
            //ROS_ERROR("no pub");
            // You can also call ros::shutdown() if you want to stop the node
        }
    }


geometry_msgs::PoseStamped gnss_pose_msg;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        gnss_pose_msg = *msg;
        if (transforms.empty()) {
        ROS_FATAL("未成功读取文件或文件内容为空");
        ros::shutdown();
        return;
        }

    // Convert the received pose to Eigen
        tf::Transform camera_pose_tf;
        tf::poseMsgToTF(msg->pose, camera_pose_tf);
        Eigen::Affine3d camera_pose_eigen;
        tf::transformTFToEigen(camera_pose_tf, camera_pose_eigen);
        // tf::Transform ndt_pose_tf;
        // tf::poseMsgToTF(msg->pose, ndt_pose_tf);
        // Eigen::Affine3d ndt_pose_eigen;
        // tf::transformTFToEigen(ndt_pose_tf, ndt_pose_eigen);

   
    // Initialize variables for closest pose
        int closest_index = -1;
        float min_distance = std::numeric_limits<float>::max();

    // Find the index of the closest pose
        for (size_t i = 0; i < transforms.size(); ++i) {
            const auto& transform = transforms[i];
            float d = distance(transform.position, {static_cast<float>(camera_pose_eigen.translation().x()),
                                                    static_cast<float>(camera_pose_eigen.translation().y()),
                                                    static_cast<float>(camera_pose_eigen.translation().z())});
            if (d < min_distance) {
                min_distance = d;
                closest_index = i;
            }
        }

        if (closest_index == -1) {
            ROS_FATAL("无法找到最近的姿态");
            // ros::shutdown();
            // return;
        }else
        {

        // Apply transformation only to the closest pose
            const auto& closest_transform = transforms[closest_index];
            Eigen::Affine3d transform_eigen;
            transform_eigen.translation() << closest_transform.translation[0], closest_transform.translation[1], closest_transform.translation[2];
            transform_eigen.linear() << closest_transform.rotation[0], closest_transform.rotation[1], closest_transform.rotation[2],
                                        closest_transform.rotation[3], closest_transform.rotation[4], closest_transform.rotation[5],
                                        closest_transform.rotation[6], closest_transform.rotation[7], closest_transform.rotation[8];
                                        
        
        // Transform the pose
            Eigen::Affine3d cam_transformed = camera_pose_eigen * transform_eigen;

        // Get the transformed position
            std::vector<float> received_pose{static_cast<float>(cam_transformed.translation().x()),
                                                static_cast<float>(cam_transformed.translation().y()), 
                                                static_cast<float>(cam_transformed.translation().z())};
        
        // std::vector<float> original_pose{0.0f, 0.0f, 0.0f}; // Origin coordinates
            float x = static_cast<float>(cam_transformed.translation().x());
            float y = static_cast<float>(cam_transformed.translation().y());
            float z = static_cast<float>(cam_transformed.translation().z());
            static std::vector<float> previous_pose{x, y, z};

            //static std::vector<float> previous_pose{cam_transformed.translation().x(), cam_transformed.translation().y(), cam_transformed.translation().z()};
            //上一次的gps值
           

            float diff_distance = distance(previous_pose, received_pose);

            
        // Define a threshold for the maximum allowed difference
            float max_difference = 0.5f; // Adjust this value as needed
            // std::cout << "Difference: " << diff_distance << std::endl;

            

        // Publish the transformed pose
            // Publish the transformed pose only if the difference is below the threshold
            if(status_msg.status.status >= 3 && status_msg.status.service > 60 ){
                std::vector<float>  now_pose{cam_pose_msg.pose.position.x, cam_pose_msg.pose.position.y, cam_pose_msg.pose.position.z};
                //std::cout<<"gnss_now"<< now_pose[0] << " " << now_pose[1] << " " << now_pose[2] << std::endl;
                float error_dis = distance(previous_pose, now_pose);
                bool  first_frame = true;
                //std::cout <<"dis:" << error_dis <<std::endl;
                if(error_dis < DISTANCE_THRESHOLD || first_frame){
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header = msg->header;
                    pose_msg.header.stamp =  ros::Time::now();
                    pose_msg.header.frame_id = "use_gnss";
                    pose_msg.pose.position.x = cam_pose_msg.pose.position.x;
                    pose_msg.pose.position.y = cam_pose_msg.pose.position.y;
                    pose_msg.pose.position.z = cam_pose_msg.pose.position.z;

                    pose_msg.pose.orientation.x = cam_pose_msg.pose.orientation.x;
                    pose_msg.pose.orientation.y = cam_pose_msg.pose.orientation.y;
                    pose_msg.pose.orientation.z = cam_pose_msg.pose.orientation.z;
                    pose_msg.pose.orientation.w = cam_pose_msg.pose.orientation.w;
                    received_pose_pub.publish(pose_msg);
                    previous_pose = {pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z};
                    //std::cout << "gnss_pre:"<< previous_pose[0] << " " << previous_pose[1] << " " << previous_pose[2] << std::endl;
                    ROS_INFO ("use_gps");
                    if (first_frame) {
                        first_frame = false;
                    }

                    //发布经纬度
                    sensor_msgs::NavSatFix long_lat_pose;
                    float x1 = cam_pose_msg.pose.position.x;
                    float y1 = cam_pose_msg.pose.position.y;
                    float z1 = cam_pose_msg.pose.position.z;
                    long_lat_pose.latitude =LAT_START + y1*LSB_M_TO_LAT_LONG;
                    long_lat_pose.longitude =LONG_START + x1*LSB_M_TO_LAT_LONG;
                    long_lat_pose.altitude = ALT_START + z1;
                    long_lat_pose.header = msg->header;
                    long_lat_pose.header.frame_id = "map";
                    long_lat_pose.header.stamp = ros::Time::now();
                    lat_long_pose_pub.publish(long_lat_pose);


                    tf::StampedTransform received_transform;
                    received_transform.stamp_ =ros::Time::now();
                    received_transform.frame_id_ = "map";  // 假设 received_pose 在 map 坐标系下
                    received_transform.child_frame_id_ = "received_pose";
                    received_transform.setOrigin(tf::Vector3(cam_pose_msg.pose.position.x, cam_pose_msg.pose.position.y, cam_pose_msg.pose.position.z));
                    received_transform.setRotation(tf::Quaternion(cam_pose_msg.pose.orientation.x,  cam_pose_msg.pose.orientation.y,cam_pose_msg.pose.orientation.z, cam_pose_msg.pose.orientation.w));
                    tf_broadcaster.sendTransform(received_transform);
                }else{
                    ROS_INFO("GPS is not well");
                }

            }else{
                if (fabs(diff_distance) < max_difference && min_ != diff_distance) {
                    std::cout <<"ookokoko"<<std::endl;
                    //previous_pose = received_pose;
                    Eigen::Quaterniond transformed_orientation(cam_transformed.linear());
                    geometry_msgs::PoseStamped pose_msg;
                    //geometry_msgs::PoseStamped ndt_pose_msg;
                    pose_msg.header = msg->header;
                    pose_msg.header.frame_id = "use_camera";
                    pose_msg.header.stamp =  ros::Time::now();
                    pose_msg.pose.position.x = static_cast<float>(SlidingWindow_Flitter(FLITTER_LENGTH, &flitter_length_cnt_x, flitter_buf_x, received_pose[0]));
                    pose_msg.pose.position.y = static_cast<float>(SlidingWindow_Flitter(FLITTER_LENGTH, &flitter_length_cnt_y, flitter_buf_y, received_pose[1]));
                    pose_msg.pose.position.z = static_cast<float>(SlidingWindow_Flitter(FLITTER_LENGTH, &flitter_length_cnt_z, flitter_buf_z, received_pose[2]));
                    
                    pose_msg.pose.orientation.x =gnss_pose_msg.pose.orientation.x;
                    pose_msg.pose.orientation.y = gnss_pose_msg.pose.orientation.y;
                    pose_msg.pose.orientation.z =gnss_pose_msg.pose.orientation.z;
                    pose_msg.pose.orientation.w = gnss_pose_msg.pose.orientation.w;

                    std::vector<float>  now_pose{pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z};
                    //std::cout << "cam_now: " << now_pose[0] << " " << now_pose[1] << " " << now_pose[2] << std::endl;
                    //std::cout << "cam_pre:"<< previous_pose[0] << " " << previous_pose[1] << " " << previous_pose[2] << std::endl;
                    float error_dis = distance(previous_pose, now_pose);
                    if(error_dis < DISTANCE_THRESHOLD ){

                        received_pose_pub.publish(pose_msg);
                        previous_pose = {pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z};

                        ROS_INFO("use_camera");

                        //发布经纬度
                        sensor_msgs::NavSatFix long_lat_pose;
                        float x2 = pose_msg.pose.position.x;
                        float y2 = pose_msg.pose.position.y;
                        float z2 = pose_msg.pose.position.z;
                        long_lat_pose.latitude =LAT_START + y2*LSB_M_TO_LAT_LONG;
                        long_lat_pose.longitude =LONG_START + x2*LSB_M_TO_LAT_LONG;
                        long_lat_pose.altitude = ALT_START + z2;
                        long_lat_pose.header = msg->header;
                        long_lat_pose.header.frame_id = "map";
                        long_lat_pose.header.stamp = ros::Time::now();
                        lat_long_pose_pub.publish(long_lat_pose);


                        tf::StampedTransform received_transform;
                        received_transform.stamp_ =ros::Time::now();
                        received_transform.frame_id_ = "map";  // 假设 received_pose 在 map 坐标系下
                        received_transform.child_frame_id_ = "received_pose";
                        received_transform.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y,  pose_msg.pose.position.z));
                        received_transform.setRotation(tf::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
                        tf_broadcaster.sendTransform(received_transform);
                    }





            

                } else {
                    previous_pose = received_pose;
                    ROS_WARN("distance is too bigger");
                    }

            }
            
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_pose_converter");
    std::shared_ptr<PoseTransformer> transformer = std::make_shared<PoseTransformer>();
    ros::spin();
    return 0;
}