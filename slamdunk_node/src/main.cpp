#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//YAML Dateien einlesen
#include "yaml-cpp/yaml.h"

namespace
{
    class Context
    {
    public:
        Context(ros::NodeHandle &n);//:m_rosNode(n){};

        void setCameraInfo(sensor_msgs::CameraInfo &left, sensor_msgs::CameraInfo &right);

        //void setKalamosContext(kalamos::Context* c);
        void tick();

        //Publishermethoden
        void publishStaticTfs();
        //void onStereoYuvData(kalamos::StereoYuvData const& stereoYuvData);

    private: //Objekte
        ros::NodeHandle& m_rosNode;
        //kalamos::Context* m_kaĺamosContext=nullptr;
        sensor_msgs::CameraInfo m_leftCamInfo;
        sensor_msgs::CameraInfo m_rightCamInfo;

        //Publisher
        //Linkes und rechtes Kamerabild(nicht rectified)
        image_transport::CameraPublisher m_PubLeftRGB;
        image_transport::CameraPublisher m_PubRightRGB;

    private: //Methoden
        //void leftRGBPublish(cv:Mat const& rgbData, std::uint64_t ts);
        //void rightRGBPublish(cv:Mat const& rgbData, std::uint64_t ts);
    };

    //Constructor mit Memberinitialisationlist
    Context::Context(ros::NodeHandle &n):
    m_rosNode(n),
    m_PubLeftRGB(image_transport::ImageTransport(n).advertiseCamera("left_rgb/image", 1)),
    m_PubRightRGB(image_transport::ImageTransport(n).advertiseCamera("right_rgb/image", 1))
    {
        

    }

    void Context::setCameraInfo(sensor_msgs::CameraInfo &left, sensor_msgs::CameraInfo &right){
        m_leftCamInfo = left;
        m_rightCamInfo = right;
        std::cout << m_leftCamInfo.width << std::endl;;
    }
    /*
    void Context::setKalamosContext(kalamos::Context* c){
        m_kalamosContext=c;
    }*/

    void Context::tick(){
        publishStaticTfs();
    }

    void Context::publishStaticTfs(){
        ros::Time time = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> vecTranforms;

        //Relation von base_link zur linken Kamera
        {
            geometry_msgs::TransformStamped transform;
            //Header
            transform.header.frame_id ="base_link";
            transform.header.stamp = time;
            transform.child_frame_id = "cam_left";

            //Zur Liste hinzufuegen
            vecTranforms.push_back(transform);
        }

        //Relation von base_link zur rechten Kamera
        {
            geometry_msgs::TransformStamped transform;
            //Header
            transform.header.frame_id ="base_link";
            transform.header.stamp = time;
            transform.child_frame_id = "cam_right";

            //Zur Liste hinzufuegen
            vecTranforms.push_back(transform);
        }
    }

    /*
    void onStereoYuvData(kalamos::StereoYuvData const& stereoYuvData){
        leftRGBPublish(stereoYuvData.leftYuv, stereoYuvData.ts);
        rightRGBPublish(stereoYuvData.rightYuv, stereoYuvData.ts);
    }

    void leftRGBPublish(cv::Mat const& rgbData, std::uint64_t ts){
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam0";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(m_leftCamInfo);

        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;

        m_PubLeftRGB.publish(image_bridge.toImageMsgs(), camInfo);
    }

    void rightRGBPublish(cv::Mat const& rgbData, std::uint64_t ts){
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam1";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(m_rightCamInfo);

        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;

        m_PubRightRGB.publish(image_bridge.toImageMsgs(), camInfo);
    }
    */

}

void yamlToCameraInfo(std::string file, sensor_msgs::CameraInfo &camInfo)
{

    YAML::Node camera_info_yaml = YAML::LoadFile(file + ".yaml");
    //Werte aus Datei in Msg packen
    //std::cout << "HIER: " << camera_info_yaml["image_width"] << std::endl;
    
    camInfo.width = camera_info_yaml["image_width"].as<uint32_t>();
    camInfo.height = camera_info_yaml["image_height"].as<uint32_t>();
    camInfo.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();
    camInfo.D = camera_info_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
    //Node Ref zu Maps
    YAML::Node camMatrix = camera_info_yaml["camera_matrix"]["data"];
    YAML::Node rectMatrix = camera_info_yaml["rectification_matrix"]["data"];
    YAML::Node projMatrix = camera_info_yaml["projection_matrix"]["data"];
    int arrayIdx = 0;
    for (auto it = camMatrix.begin(); it != camMatrix.end(); ++it)
    {
        camInfo.K[arrayIdx++] = it->as<double>();
    }
    arrayIdx = 0;
    for (auto it = rectMatrix.begin(); it != rectMatrix.end(); ++it)
    {
        camInfo.R[arrayIdx++] = it->as<double>();
    }
    arrayIdx = 0;
    for (auto it = projMatrix.begin(); it != projMatrix.end(); ++it)
    {
        camInfo.P[arrayIdx++] = it->as<double>();
    }
}

int main(int ac, char** av){
    ros::init(ac, av, "slamdunk_node_Simon");

    ros::NodeHandle n;
    
    Context context(n);

    sensor_msgs::CameraInfo cam_left;
    sensor_msgs::CameraInfo cam_right;

    //CameraInfo aus yaml Datei laden
    //yamlToCameraInfo("/home/slamdunk/ws_slamdunk/src/slamdunk_node/camera_data/left", context.leftInfo);
    //yamlToCameraInfo("/home/slamdunk/ws_slamdunk/src/slamdunk_node/camera_data/right", context.rightInfo);
    yamlToCameraInfo("/home/simon/Dokumente/ws_slamdunk/src/slamdunk_node/camera_data/left", cam_left);
    yamlToCameraInfo("/home/simon/Dokumente/ws_slamdunk/src/slamdunk_node/camera_data/right", cam_right);

    context.setCameraInfo(cam_left, cam_right);
    //kalamos::Callbacks kalamosCbs;
    //kalamosCbs.period = 30;

    //kalamosCbs.periodicCallback = std::bind(&Context::tick, &context);
    //kalamosCbs.stereoYuvCallback = std::bind(&Context::onStereoYuvData, &context, std::placeholders::_1);

    //std::unique_ptr<kalamos::Context> kalamosContext = kalamos::init(kalamosCbs);
    //if(kalamosContext!=nullptr){
    //    context.setKalamosContext(kalamosContext.get());  
    //      kalamosContext->run();  
    //}

    ros::Rate looptime(1);
    
    while(ros::ok()){
        std::cout << "RUNNING" << std::endl;
        ros::spinOnce();
        looptime.sleep();
    }

    return 0;
}