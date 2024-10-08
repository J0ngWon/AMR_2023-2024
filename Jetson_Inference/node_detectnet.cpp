// ROS 관련 헤더
#include "ros_compat.h"
#include <ros/ros.h>
#include "image_converter.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

// OpenCV 관련 헤더
#include <opencv2/opencv.hpp>

// Jetson Inference 관련 헤더
#include <jetson-inference/detectNet.h>

// C++ 표준 라이브러리
#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>

// Jetson Inference 네트워크 설정
detectNet* net = NULL;
uint32_t overlay_flags = detectNet::OVERLAY_NONE;

// 이미지 변환기
imageConverter* input_cvt   = NULL;
imageConverter* overlay_cvt = NULL;

// 이미지 퍼블리셔
image_transport::Publisher depth_image_pub;
image_transport::Publisher rgb_image_pub;

// ROS 퍼블리셔
Publisher<vision_msgs::Detection2DArray> detection_pub = NULL;
Publisher<sensor_msgs::Image> overlay_pub = NULL;
Publisher<vision_msgs::VisionInfo> info_pub = NULL;
Publisher<visualization_msgs::Marker> marker_pub = NULL;
Publisher<std_msgs::Float64MultiArray> pick_goal_pub = NULL;
Publisher<std_msgs::String> web_msg_pub = NULL;
Publisher<std_msgs::String> web_state_pub = NULL;

std_msgs::Float64MultiArray pick_goal_msg;
std_msgs::String web_msg;
std_msgs::String web_state;
char msg_buffer[100];

const float f_x = 594.21434211923247;  // X 축 초점 거리
const float f_y = 591.04053696870778;  // Y 축 초점 거리
const float c_x = 339.30780975300314;  // 광학 중심 X
const float c_y = 242.73913761751615;  // 광학 중심 Y

// 비전 정보 메시지
vision_msgs::VisionInfo info_msg;

// 바운딩 박스의 좌표 점
float cx, cy;
uint16_t depth_value;

// 목적지 상태 값
int current_step = 0;
std::string current_state;
actionlib_msgs::GoalStatus current_status;
actionlib_msgs::GoalStatus pre_status;

// 거리 계산 변수 (map으로부터 물체까지)
double map_x, map_y, map_z;

// 인식 조건 값
const float MIN_CONFIDENCE = 0.5f;
std::string TARGET_OBJECT = "can";
int consistent_count = 0;
const int MIN_CONSISTENT_FRAMES = 500;
const ros::Duration MAX_INACTIVE_TIME(1.0);  // 1초
float err = 40.0;
int marker_id = 0;

visualization_msgs::Marker marker;

struct DetectedObject {
    std::string class_name;
    float confidence;
    float cx, cy, depth;
    int consistent_count;
    bool set_goal;
    int marker_id;
    ros::Time last_seen;
};

std::vector<DetectedObject> detected_objects;

// for test
bool test = true;

// triggered when a new subscriber connected
void info_callback() {
    ROS_INFO("new subscriber connected to vision_info topic, sending VisionInfo msg");
    info_pub->publish(info_msg);
}

// 시나리오 단계 업데이트
void updateState() {
    current_step = current_step + 1;

    if (current_step == 5) {
        current_step = 0;
    }

    switch (current_step) {
        case 0:
            current_state = "READY";
            break;
        case 1:
            current_state = "CAMERA ON";
            break;
        case 2:
            current_state = "CAMERA OFF";
            break;
        case 3:
            current_state = "PICK";
            break;
        case 4:
            current_state = "HOME";
            break;
        default:
            current_state = "READY";
            break;
    }

    web_state.data = current_state;
    web_state_pub->publish(web_state);
    ROS_INFO("현재 단계 : %s", current_state.c_str());
}

// web 메시지 발행
void publishMessage(const std::string& str) {
    std_msgs::String msg;
    msg.data = str;

    // 메시지 발행
    web_msg_pub->publish(msg);
}

// 목적지 도착 여부
void goalState_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    // 상태 목록이 비어있는지 확인
    if (msg->status_list.empty()) {
        return;
    }

    // 최신 상태 가져오기
    current_status = msg->status_list.back();

    // 상태가 이전과 다를 경우에만 처리
    if (current_status.status != pre_status.status) {
        switch (current_status.status) {
            case 1:
                ROS_INFO("목표를 향해 이동 중입니다.");
                break;
            case 3:
                ROS_INFO("목표 지점에 도착했습니다!");
                updateState();
                break;
            case 4:
                ROS_WARN("목표 도달에 실패했습니다.");
                break;
            default:
                ROS_INFO("현재 목표 상태: %d", current_status.status);
                break;
        }
    }

    pre_status = current_status;
}


void setObject_Callback(const std_msgs::String::ConstPtr& msg) {
    std::vector<std::string> object_list = {"can", "bottle"};

    bool found = false;
    std::string object = msg->data;

    for (const std::string& item : object_list) {
        if (item == object) {
            found = true;

            TARGET_OBJECT = item;
            
            std::snprintf(msg_buffer, sizeof(msg_buffer), "물체가 설정 되었습니다.");
            publishMessage(std::string(msg_buffer));

            break;
        } 
    }
    if (!found) {
        std::snprintf(msg_buffer, sizeof(msg_buffer), "등록되지 않은 물체입니다.");
        publishMessage(std::string(msg_buffer));
    }
}


// Marker 생성
void createMarker(visualization_msgs::Marker& marker, int marker_id, float depth, float width, float height)
{
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // 위치 값 설정
    marker.pose.position.x = depth;
    marker.pose.position.y = width;
    marker.pose.position.z = height;

    // 고정된 회전 값 설정 (필요에 따라 변경 가능)
    double rx = 0.0; // 원하는 roll 값
    double ry = 0.0; // 원하는 pitch 값
    double rz = 0.0; // 원하는 yaw 값 (라디안 값)

    // Euler angles (rx, ry, rz)를 quaternion으로 변환
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(rx, ry, rz);
    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    // 크기 설정
    marker.scale.x = 0.055;
    marker.scale.y = 0.155;
    marker.scale.z = 0.05;

    // 색상 설정
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

// source_frame <-> map frame 길이 변환
std::tuple<double, double, double> transformMarkerToMap(const std::string& source_frame, const std::string& target_frame, double x, double y, double z) {
    // Create a TF listener
    tf::TransformListener listener;

    geometry_msgs::PoseStamped marker_pose;
    marker_pose.header.frame_id = source_frame;
    marker_pose.pose.position.x = x;
    marker_pose.pose.position.y = y;
    marker_pose.pose.position.z = z;
    marker_pose.pose.orientation.w = 1.0;

    try {
        // 변환 시간 대기
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));

        // Transform the pose to map frame
        geometry_msgs::PoseStamped map_pose;
        listener.transformPose(target_frame, marker_pose, map_pose);

        // Extract the transformed coordinates
        double goal_x = map_pose.pose.position.x;
        double goal_y = map_pose.pose.position.y;
        double goal_z = map_pose.pose.position.z;

        // Return the transformed coordinates as a tuple
        return std::make_tuple(goal_x, goal_y, goal_z);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("TF transformation error: %s", ex.what());
        // Return a default value or error indicator
        return std::make_tuple(0.0, 0.0, 0.0);
    }
}

// base_link <-> 물체 길이 변환
geometry_msgs::Point transformPointToBaseFootprint(double map_x, double map_y, double map_z) {
    // Create a TF listener
    tf::TransformListener listener;

    // Define a point in the map frame
    geometry_msgs::PointStamped map_point;
    map_point.header.frame_id = "map";
    map_point.point.x = map_x;
    map_point.point.y = map_y;
    map_point.point.z = map_z;

    // Define a point to store the transformed point
    geometry_msgs::PointStamped base_point;

    try {
        // Wait for the transform to be available
        listener.waitForTransform("arm_link", "map", ros::Time(0), ros::Duration(3.0));
        
        // Transform the point to the base_footprint frame
        listener.transformPoint("arm_link", map_point, base_point);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }

    // Return the transformed point in base_footprint frame
    return base_point.point;
}

void moveToObject(double x, double y) {
    // move_base 액션 서버와 통신하는 클라이언트 생성
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // move_base 서버가 시작될 때까지 대기
    ROS_INFO("Waiting for the move_base action server to start...");
    ac.waitForServer(ros::Duration(5.0));
    ROS_INFO("Connected to move_base server");

    // 목표 생성
    move_base_msgs::MoveBaseGoal goal;

    // frame_id를 "base_footprint"로 설정하여 로봇의 현재 위치 기준으로 목표 설정
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();

    // x, y 좌표를 입력받아 목표 좌표 설정
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0; 
    goal.target_pose.pose.orientation.w = 1.0;  // 회전 없이 직진

    // 목표를 전송
    ac.sendGoal(goal);

    // 결과를 기다림
    ac.waitForResult();

    // 결과 확인
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        pick_goal_msg.data.clear();

        //ROS_INFO("ARM reached the goal!");
        std::snprintf(msg_buffer, sizeof(msg_buffer), "목적지 도착 완료");
        publishMessage(std::string(msg_buffer));

        // 로봇 팔로부터 물체까지 거리계산
        geometry_msgs::Point base_point = transformPointToBaseFootprint(map_x, map_y, map_z);

        double arm_x = base_point.x;
        double arm_y = base_point.y;
        double arm_z = base_point.z;

        //ROS_INFO("로봇팔이 집을 거리 : X: %fcm, Y: %fcm, Z: %fcm (고정 9cm)", r1*100, r2*100, r3*100);
        std::snprintf(msg_buffer, sizeof(msg_buffer), "로봇팔이 집을 거리 : X: %fcm, Y: %fcm, Z: %fcm (고정 9cm)", arm_x*100, arm_y*100, arm_z*100);
        publishMessage(std::string(msg_buffer));

        std::string confirm2;
        float preset_x2, preset_y2, preset_z2;
        std::cout << "Did you Confirm? (y/n): ";
        std::getline(std::cin, confirm2);
        if (confirm2 == "y") {
            pick_goal_msg.data.push_back(arm_x);
            pick_goal_msg.data.push_back(arm_y);
            pick_goal_msg.data.push_back(0.09);
    
            pick_goal_pub->publish(pick_goal_msg);
        }
        else {
            std::cout << "Enter x, y, z: ";
            std::cin >> preset_x2 >> preset_y2 >> preset_z2;
            
            pick_goal_msg.data.push_back(preset_x2);
            pick_goal_msg.data.push_back(preset_y2);
            pick_goal_msg.data.push_back(preset_z2);
    
            pick_goal_pub->publish(pick_goal_msg);
        }
    }
    else
        //ROS_INFO("The TurtleBot3 failed to reach the goal.");
        std::snprintf(msg_buffer, sizeof(msg_buffer), "목적지 도착 실패");
        publishMessage(std::string(msg_buffer));
}

// 객체 필터링
bool image_filtering(const detectNet::Detection* det) {
    std::string detected_class = net->GetClassDesc(det->ClassID);
    float confidence = det->Confidence;
    det->Center(&cx, &cy);
    ros::Time current_time = ros::Time::now();

    // 오래된 객체 검수 후 삭제
    auto it = std::remove_if(detected_objects.begin(), detected_objects.end(),
        [current_time](const DetectedObject& obj) {
            if ((current_time - obj.last_seen) > MAX_INACTIVE_TIME) {
                ROS_INFO("Removing object: %s at (%f, %f, %f) with confidence %f", 
                        obj.class_name.c_str(), obj.cx, obj.cy, obj.depth, obj.confidence);

                // 마커 삭제를 위한 Marker 메시지 생성
                visualization_msgs::Marker delete_marker;
                delete_marker.header.frame_id = "camera_link";
                delete_marker.header.stamp = ros::Time::now();
                delete_marker.ns = "object";
                delete_marker.id = obj.marker_id;
                delete_marker.action = visualization_msgs::Marker::DELETE;

                // 마커 퍼블리시
                marker_pub->publish(delete_marker);

                return true;  // 삭제 조건 충족
            }
            return false;
        });
    // 벡터에서 객체 삭제
    detected_objects.erase(it, detected_objects.end());

    if (detected_class == TARGET_OBJECT && confidence >= MIN_CONFIDENCE) {
        bool object_found = false;
        
        for (auto& obj_item : detected_objects) {
            //유클리드 거리
            float distance = sqrt(pow(obj_item.cx - cx, 2) + pow(obj_item.cy - cy, 2));
            
            if (distance < err) {
                obj_item.consistent_count++;
                obj_item.depth = obj_item.depth + depth_value;
                obj_item.last_seen = current_time;
                object_found = true;

                if (obj_item.consistent_count >= MIN_CONSISTENT_FRAMES && !obj_item.set_goal) {
                    float width, height;

                    obj_item.depth = obj_item.depth / MIN_CONSISTENT_FRAMES;

                    // 다음 단계 시작
                    updateState();

                    test = false;
                    
                    //방법 1
                    //width = (cx - 640 / 2) * (depth_value - 0) * 0.001;
                    //height = (cy - 480 / 2) * (depth_value - 0) * 0.001;

                    //방법 2
                    width = -((obj_item.cx - c_x) * obj_item.depth) / f_x * 0.001;
                    height = ((obj_item.cy - c_y) * obj_item.depth) / f_y * 0.001;

                    std::snprintf(msg_buffer, sizeof(msg_buffer), "카메라로 부터 물체까지 : depth:%fcm width:%fcm height:%fcm", obj_item.depth*0.1, width*100, height*100);
                    publishMessage(std::string(msg_buffer));

                    //ROS_INFO("object distance : depth:%fcm width:%fcm height:%fcm", obj_item.depth*0.1, width*100, height*100);

                    createMarker(marker, obj_item.marker_id, obj_item.depth*0.001, width, height);
                    marker_pub->publish(marker);

                    // 좁은 y 범위 제한
                    if (width >= -0.06 && width <= 0.06) {
                        ROS_INFO("-6cm<= %f <=6cm is [y = 0]", width);
                        width = 0;
                    }

                    // 인식된 물체를 base_footprint 기준 거리 계산
                    auto [goal_x, goal_y, goal_z] = transformMarkerToMap("camera_link", "base_footprint", obj_item.depth*0.001, width, height);
                    //ROS_INFO("robot_center to object : X: %fm, Y: %fm, Z: %fm", goal_x , goal_y, goal_z);
                    std::snprintf(msg_buffer, sizeof(msg_buffer), "로봇 중심부터 물체까지 : X: %fm, Y: %fm, Z: %fm", goal_x , goal_y, goal_z);
                    publishMessage(std::string(msg_buffer));

                    // 인식된 물체를 map 기준 거리 계산
                    std::tie(map_x, map_y, map_z) = transformMarkerToMap("camera_link", "map", obj_item.depth * 0.001, width, height);
                    //ROS_INFO("map to object X: %fm, Y: %fm, Z: %fm", map_x, map_y, map_z);

                    // 목적지 선정된 물체 인증
                    obj_item.set_goal = true;
                    
                    //목적지 설정 0.255 -> 차량 중심에서 전방까지 거리 | 0.3 -> 차량 전방에서 물체까지의 거리 설정 [m]
                    //ROS_INFO("move to object : X: %f Y: %f", goal_x - 0.3, goal_y);
                    std::snprintf(msg_buffer, sizeof(msg_buffer), "물체까지 로봇 이동거리 : X: %f Y: %f", goal_x - (0.255 + 0.3), goal_y);
                    publishMessage(std::string(msg_buffer));

                    // 데이터 검수
                    std::string confirm;
                    float preset_x, preset_y;
                    std::cout << "Did you Confirm? (y/n): ";
                    std::getline(std::cin, confirm);

                    if (confirm == "y") {
                        moveToObject(goal_x - (0.3 + 0.255), goal_y);
                    }
                    else {
                        std::cout << "Enter x, y: ";
                        std::cin >> preset_x >> preset_y;
                        moveToObject(preset_x, preset_y);
                    }

                    return true;
                }
                break;
            }
        }

        if (!object_found && (depth_value != 0)) {
            DetectedObject obj;

            obj.class_name = detected_class;
            obj.confidence = confidence;
            obj.cx = cx;
            obj.cy = cy;
            obj.depth = depth_value;
            obj.consistent_count = 1;
            obj.set_goal = false;
            obj.marker_id = marker_id++;
            obj.last_seen = current_time;   
            detected_objects.push_back(obj);
            ROS_INFO("New Object!");

            for (const auto& obj : detected_objects) {
                ROS_INFO("Detected Object: %s at (%f, %f, %f) with confidence %f, ID: %d", obj.class_name.c_str(), obj.cx, obj.cy, obj.depth, obj.confidence, obj.marker_id);
            }
        }
    } 
    return true;
}

// publish overlay image
bool publish_overlay(detectNet::Detection* detections, int numDetections)
{
    // get the image dimensions
    const uint32_t width = input_cvt->GetWidth();
    const uint32_t height = input_cvt->GetHeight();

    // assure correct image size
    if (!overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat))
        return false;

    // generate the overlay
    if (!net->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height,
                      imageConverter::InternalFormat, detections, numDetections, overlay_flags))
    {
        return false;
    }

    // populate the message
    sensor_msgs::Image msg;

    if (!overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat))
        return false;

    // populate timestamp in header field
    msg.header.stamp = ROS_TIME_NOW();

    // publish the message    
    overlay_pub->publish(msg);
    ROS_DEBUG("publishing %ux%u overlay image", width, height);

    return true;
}

// input image subscriber callback
void img_callback(const sensor_msgs::ImageConstPtr& input)
{
    // convert the image to reside on GPU
    if (!input_cvt || !input_cvt->Convert(input))
    {
        ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
        return;    
    }

    web_state.data = "READY";
    web_state_pub->publish(web_state);

    if (test)
    //if (current_state == "CAMERA ON") 
    {
        // classify the image
        detectNet::Detection* detections = NULL;
        const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

        // verify success    
        if (numDetections < 0)
        {
            ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
            return;
        }

        // if objects were detected, send out message
        if (numDetections > 0)
        {
            // create a detection for each bounding box
            vision_msgs::Detection2DArray msg;

            for (int n = 0; n < numDetections; n++)
            {
                detectNet::Detection* det = detections + n;
                
                // create a detection sub-message   
                vision_msgs::Detection2D detMsg;

                detMsg.bbox.size_x = det->Width();
                detMsg.bbox.size_y = det->Height();
                
                detMsg.bbox.center.x = cx;
                detMsg.bbox.center.y = cy;
                detMsg.bbox.center.theta = 0.0f;        // TODO optionally output object image

                if (depth_value != 0) {
                    image_filtering(det);
                }
                // create classification hypothesis
                vision_msgs::ObjectHypothesisWithPose hyp;
                
                detMsg.results.push_back(hyp);
                msg.detections.push_back(detMsg);
            }

            // populate timestamp in header field
            msg.header.stamp = ROS_TIME_NOW();

            // publish the detection message
            detection_pub->publish(msg);
        }

        // generate the overlay (if there are subscribers)8
        if (ROS_NUM_SUBSCRIBERS(overlay_pub) > 0)
            //publish_overlay(detections, numDetections); //화면에 이미지 오버레이
        
        
        if (rgb_image_pub.getNumSubscribers() > 0)
        {
            // 오버레이 생성
            if (!net->Overlay(input_cvt->ImageGPU(), input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(),
                              imageConverter::InternalFormat, detections, numDetections, overlay_flags))
            {
                ROS_ERROR("failed to generate overlay on image");
                return;
            }

            // 오버레이된 이미지를 ROS 메시지로 변환
            sensor_msgs::Image msg;
            if (!input_cvt->Convert(msg, imageConverter::ROSOutputFormat))
            {
                ROS_ERROR("failed to convert overlaid image to ROS message");
                return;
            }

            // 타임스탬프 추가
            msg.header.stamp = ROS_TIME_NOW();

            // 오버레이된 이미지 발행
            rgb_image_pub.publish(msg);
            ROS_DEBUG("publishing %ux%u overlaid RGB image", input_cvt->GetWidth(), input_cvt->GetHeight());
        }
    }
    else
    {
        //ROS_INFO("Object detection is currently disabled.");
    }
}

// Depth 이미지 콜백 함수
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;

        // 중앙 깊이 값 계산
        int centerX = depth_image.cols / 2;
        int centerY = depth_image.rows / 2;
        if (centerX < 0 || centerX >= depth_image.cols || centerY < 0 || centerY >= depth_image.rows)
        {
            ROS_ERROR("Depth image dimensions are invalid");
            return;
        }
        depth_value = depth_image.at<uint16_t>(cy, cx);
        //ROS_INFO("Depth Value: %d", depth_value);
        
        // 깊이 영상 토픽 발행 Depth 이미지를 8비트로 변환하여 전송
        cv::Mat adjMap;
        double min, max;
        cv::minMaxIdx(depth_image, &min, &max);
        cv::convertScaleAbs(depth_image, adjMap, 255 / max); // 8비트로 변환

        // 깊이 영상 표시
        //cv::imshow("Depth Image", adjMap);
        //cv::waitKey(1);

        // 변환된 8비트 Depth 이미지를 퍼블리시
        //sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", adjMap).toImageMsg();
        //depth_image_pub.publish(depth_msg);

         // 깊이 이미지의 Base64 인코딩 및 발행
        //std::vector<uchar> buf;
        //cv::imencode(".png", adjMap, buf); // PNG 형식으로 이미지를 인코딩
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", msg->encoding.c_str());
    }
    catch (cv::Exception& e)
    {
        ROS_ERROR("OpenCV Error: %s", e.what());
    }
}

// node main loop
int main(int argc, char **argv)
{
    /*
     * create node instance
     */
    ros::init(argc, argv, "detectnet");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /*
     * retrieve parameters
     */    
    std::string model_name  = "ssd-mobilenet-v2";
    std::string model_path;
    std::string prototxt_path;
    std::string class_labels_path;
    
    std::string input_blob  = DETECTNET_DEFAULT_INPUT;
    std::string output_cvg  = DETECTNET_DEFAULT_COVERAGE;
    std::string output_bbox = DETECTNET_DEFAULT_BBOX;
    std::string overlay_str = "box,labels,conf";

    float mean_pixel = 0.0f;
    float threshold  = DETECTNET_DEFAULT_THRESHOLD;

    ROS_DECLARE_PARAMETER("model_name", model_name);
    ROS_DECLARE_PARAMETER("model_path", model_path);
    ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
    ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
    ROS_DECLARE_PARAMETER("input_blob", input_blob);
    ROS_DECLARE_PARAMETER("output_cvg", output_cvg);
    ROS_DECLARE_PARAMETER("output_bbox", output_bbox);
    ROS_DECLARE_PARAMETER("overlay_flags", overlay_str);
    ROS_DECLARE_PARAMETER("mean_pixel_value", mean_pixel);
    ROS_DECLARE_PARAMETER("threshold", threshold);

    /*
     * retrieve parameters
     */
    ROS_GET_PARAMETER("model_name", model_name);
    ROS_GET_PARAMETER("model_path", model_path);
    ROS_GET_PARAMETER("prototxt_path", prototxt_path);
    ROS_GET_PARAMETER("class_labels_path", class_labels_path);
    ROS_GET_PARAMETER("input_blob", input_blob);
    ROS_GET_PARAMETER("output_cvg", output_cvg);
    ROS_GET_PARAMETER("output_bbox", output_bbox);
    ROS_GET_PARAMETER("overlay_flags", overlay_str);
    ROS_GET_PARAMETER("mean_pixel_value", mean_pixel);
    ROS_GET_PARAMETER("threshold", threshold);

    overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());

    /*
     * load object detection network
     */
    if (model_path.size() > 0)
    {
        // create network using custom model paths
        net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), 
                                mean_pixel, class_labels_path.c_str(), threshold, 
                                input_blob.c_str(), output_cvg.c_str(), output_bbox.c_str());
    }
    else
    {
        // create network using the built-in model
        net = detectNet::Create(model_name.c_str());
    }

    if (!net)
    {
        ROS_ERROR("failed to load detectNet model");
        return 0;
    }

    /*
     * create the class labels parameter vector
     */
    std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
    std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
    const size_t model_hash = model_hasher(model_hash_str);
    
    ROS_INFO("model hash => %zu", model_hash);
    ROS_INFO("hash string => %s", model_hash_str.c_str());

    // obtain the list of class descriptions
    std::vector<std::string> class_descriptions;
    const uint32_t num_classes = net->GetNumClasses();

    for (uint32_t n = 0; n < num_classes; n++)
        class_descriptions.push_back(net->GetClassDesc(n));

    // create the key on the param server
    std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

    ROS_DECLARE_PARAMETER(class_key, class_descriptions);
    ROS_SET_PARAMETER(class_key, class_descriptions);
        
    // populate the vision info msg
    std::string node_namespace = ROS_GET_NAMESPACE();
    ROS_INFO("node namespace => %s", node_namespace.c_str());

    info_msg.database_location = node_namespace + std::string("/") + class_key;
    info_msg.database_version  = 0;
    info_msg.method            = net->GetModelPath();
    
    ROS_INFO("class labels => %s", info_msg.database_location.c_str());

    /*
     * create image converter objects
     */
    input_cvt = new imageConverter();
    overlay_cvt = new imageConverter();

    if (!input_cvt || !overlay_cvt)  
    {
        ROS_ERROR("failed to create imageConverter objects");
        return 0;
    }

    //cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);

    image_transport::ImageTransport it(nh);
    
    image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

    depth_image_pub = it.advertise("/camera/depth/image_published", 1);
    rgb_image_pub = it.advertise("/camera/rgb/image_published", 1);
    //depth_base64_pub = nh.advertise<std_msgs::String>("/camera/depth_base64", 1);
    
    /*
     * advertise publisher topics
     */
    ROS_CREATE_PUBLISHER(vision_msgs::Detection2DArray, "detections", 25, detection_pub);
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
    ROS_CREATE_PUBLISHER(visualization_msgs::Marker, "object_marker", 1, marker_pub);
    ROS_CREATE_PUBLISHER(std_msgs::Float64MultiArray, "pick_goal", 10, pick_goal_pub);
    ROS_CREATE_PUBLISHER(std_msgs::String, "web_msg", 1, web_msg_pub);
    ROS_CREATE_PUBLISHER(std_msgs::String, "web_state", 1, web_state_pub);
    ROS_CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);

    /*
     * subscribe to image topic
     */
    auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
    
    // Subscribe to switch_state topic
    ros::Subscriber status_sub = nh.subscribe("move_base/status", 10, goalState_Callback);
    ros::Subscriber set_object_sub = nh.subscribe("set_object", 1, setObject_Callback);

    /*
     * wait for messages
     */
    ROS_INFO("detectnet node initialized, waiting for messages");

    ros::spin();

    /*
     * free resources
     */
    delete net;
    delete input_cvt;
    delete overlay_cvt;

    return 0;
}
