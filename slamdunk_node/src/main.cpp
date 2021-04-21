#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "kalamos_context.hpp"
#include "cvutils.hpp"

//YAML Dateien einlesen
#include "yaml-cpp/yaml.h"

namespace
{
    class Context
    {
    public:
        Context(ros::NodeHandle &n);//:m_rosNode(n){};

        void setCameraInfo(sensor_msgs::CameraInfo &left, sensor_msgs::CameraInfo &right);

        void setKalamosContext(kalamos::Context* c);
        void tick();

        //Publishermethoden
        //Publishes statische Tfs
        void publishStaticTfs();
        void stereoImgCallback(kalamos::StereoYuvData const& stereoYuvData);

    private: //Objekte
        ros::NodeHandle& m_rosNode;
        kalamos::Context* m_kalamosContext=nullptr;
	std::unique_ptr<kalamos::ServiceHandle> m_captureHandle;
        //Kamerainfo aus Yaml-Datei
        sensor_msgs::CameraInfo m_leftCamInfo;
        sensor_msgs::CameraInfo m_rightCamInfo;

        //Publisher
        //Linkes und rechtes Kamerabild(nicht rectified)
        image_transport::CameraPublisher m_PubLeftRGB;
        image_transport::CameraPublisher m_PubRightRGB;
        //Linkes und rechtes Kamerabild(korrigiert/rectified)
        image_transport::CameraPublisher m_PubLeftRGBRect;
        image_transport::CameraPublisher m_PubRightRGBRect;
        //Aenderungen von Koordinateninformationen
        tf2_ros::TransformBroadcaster m_transfromBroadcaster;

    private: //Methoden
	    //Kalamos
	    void setVideoSettings();

        //Unrectified Kamerabilderpublishermethode
        void leftRGBPublish(cv::Mat const& rgbData, std::uint64_t ts);
        void rightRGBPublish(cv::Mat const& rgbData, std::uint64_t ts);
        //Rectified Kamerabilderpublishermethode
        void leftRGBRectPublish(cv::Mat const& rgbData, std::uint64_t ts);
        void rightRGBRectPublish(cv::Mat const& rgbData, std::uint64_t ts);
    };

    //Konstruktor mit Memberinitialisationlist
    Context::Context(ros::NodeHandle &n):
    m_rosNode(n),
    m_PubLeftRGB(image_transport::ImageTransport(n).advertiseCamera("left_rgb/image", 1)),
    m_PubRightRGB(image_transport::ImageTransport(n).advertiseCamera("right_rgb/image", 1)),
    m_PubLeftRGBRect(image_transport::ImageTransport(n).advertiseCamera("left_rgb_rect/image", 1)),
    m_PubRightRGBRect(image_transport::ImageTransport(n).advertiseCamera("right_rgb_rect/image", 1))
    {
        

    }

    //Speichern der Infos aus der Kamera-Yaml-Dateien
    void Context::setCameraInfo(sensor_msgs::CameraInfo &left, sensor_msgs::CameraInfo &right){
        m_leftCamInfo = left;
        m_rightCamInfo = right;
        
    }
    
    void Context::setKalamosContext(kalamos::Context* c){
        m_kalamosContext=c;

	setVideoSettings();
    }

    void Context::setVideoSettings(){
	if(m_kalamosContext==nullptr)
		return;
	
	kalamos::VideoMode videoMode = kalamos::VideoMode::MODE_1500_1500_60;
	std::string videoModeStr("1500x1500 @ 60 FPS");
	ROS_INFO("Set video mode to %s", videoModeStr.c_str());
	m_kalamosContext->setVideoMode(videoMode);
    }

    void Context::tick(){
        publishStaticTfs();

	//Aktivieren der Kamerafuntionalität
	if(!m_captureHandle){
		m_captureHandle = m_kalamosContext->startService(kalamos::ServiceType::CAPTURE);
	}
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

            //Position und Rotation
            transform.transform.translation.x =0;
            transform.transform.translation.y = -0.1; //Abstand zum Mittelpunkt
            transform.transform.translation.z = 0;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.transform.rotation.w = 1;

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

            //Position und Rotation
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0.1;
            transform.transform.translation.z = 0;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.transform.rotation.w = 1;

            //Zur Liste hinzufuegen
            vecTranforms.push_back(transform);
        }
        //Publish Koordinaten
        m_transfromBroadcaster.sendTransform(vecTranforms);
        
    }

    
    void Context::stereoImgCallback(kalamos::StereoYuvData const& stereoYuvData){
        //Kamerabildgroesse(wird reduziert diese)
        cv::Size cropSize{1280, 720};
        //Unrectified Kamerabilder Verarbeitung
        //Yuv zu RGB umwandeln, Bildgroesse verkleinern
        cv::Mat leftRGBFrame = yuvToRgb(stereoYuvData.leftYuv, cropSize.width, cropSize.height); 
        cv::Mat rightRGBFrame = yuvToRgb(stereoYuvData.rightYuv, cropSize.width, cropSize.height);
        //Publishermethode aufrufen
        leftRGBPublish(leftRGB, stereoYuvData.ts);
        rightRGBPublish(rightRGB, stereoYuvData.ts);
        
        //Korrigierte(rectified) Kamerabildern Verarbeitung
        cv::Mat leftRGBRectFrame;
        cv::Mat rightRGBRectFrame;
        //Linkes und rechtes Frame korrigieren(Closed-source kalamos code)
        m_kalamosContext->rectifyFrame(leftRGBFrame, leftRGBRectFrame);
        m_kalamosContext->rectifyFrame(rightRGBFrame, rightRGBRectFrame);
        //Publishermethode aufrufen
        leftRGBRectPublish(leftRGBRectFrame, stereoYuvData.ts);
        rightRGBRectPublish(rightRGBRectFrame, stereoYuvData.ts); 
    }

    void Context::leftRGBPublish(cv::Mat const& rgbData, std::uint64_t ts){

        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam_left";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(&m_leftCamInfo);

        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;
	
        //Hier wird sowohl das Frame als auch ein KameraInfo-Topic veroeffentlicht
        m_PubLeftRGB.publish(img_bridge.toImageMsg(), camInfo);

    }

    void Context::rightRGBPublish(cv::Mat const& rgbData, std::uint64_t ts){

        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam_right";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(&m_rightCamInfo);


        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->header = img_bridge.header;
        //Hier wird sowohl das Frame als auch ein KameraInfo-Topic veroeffentlicht
        m_PubRightRGB.publish(img_bridge.toImageMsg(), camInfo);

    }

    void Context::leftRGBRectPublish(cv::Mat const& rgbData, std::uint64_t ts){
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam_left";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->header = img_bridge.header;
        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        camInfo->D.resize(5);

        kalamos::RectifiedFrameStaticProperties staticProps = m_kalamosContext->getLeftRectStaticProperties();

        float fx = staticProps.fx;
        float fy = staticProps.fy;
        float cx = rgbData.cols / 2.0;
        float cy = rgbData.rows / 2.0;

        camInfo->P.fill(0);
        camInfo->P[0] = fx;
        camInfo->P[2] = cx;
        camInfo->P[5] = fy;
        camInfo->P[6] = cy;
        camInfo->P[10] = 1.0;
        //Hier wird sowohl das Frame als auch ein KameraInfo-Topic veroeffentlicht
        m_PubLeftRGBRect.publish(img_bridge.toImageMsg(), camInfo);

    }
    
    void Context::rightRGBRectPublish(cv::Mat const& rgbData, std::uint64_t ts){
        cv_bridge::CvImage img_bridge;
        img_bridge.image = rgbData;
        img_bridge.encoding = sensor_msgs::image_encodings::RGB8;
        img_bridge.header.frame_id = "cam_right";
        img_bridge.header.stamp = ros::Time().fromNSec(ts);

        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo());
        camInfo->header = img_bridge.header;
        camInfo->width = rgbData.cols;
        camInfo->height = rgbData.rows;
        camInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        camInfo->D.resize(5);

        kalamos::RectifiedFrameStaticProperties staticProps = m_kalamosContext->getLeftRectStaticProperties();

        float fx = staticProps.fx;
        float fy = staticProps.fy;
        float cx = rgbData.cols / 2.0;
        float cy = rgbData.rows / 2.0;

        camInfo->P.fill(0);
        camInfo->P[0] = fx;
        camInfo->P[2] = cx;
        camInfo->P[5] = fy;
        camInfo->P[6] = cy;
        camInfo->P[10] = 1.0;
        //Hier wird sowohl das Frame als auch ein KameraInfo-Topic veroeffentlicht
        m_PubRightRGB.publish(img_bridge.toImageMsg(), camInfo);
    }
    

}

void yamlToCameraInfo(std::string file, sensor_msgs::CameraInfo &camInfo)
{

    YAML::Node camera_info_yaml = YAML::LoadFile(file + ".yaml");
    //Werte aus Datei in Msg packen
    
    
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

    //CameraInfo aus yaml Datei lesen
    yamlToCameraInfo("/home/slamdunk/ws_slamdunk/src/slamdunk_ros/slamdunk_node/camera_data/left", cam_left);
    yamlToCameraInfo("/home/slamdunk/ws_slamdunk/src/slamdunk_ros/slamdunk_node/camera_data/right", cam_right);

    context.setCameraInfo(cam_left, cam_right);

    kalamos::Callbacks kalamosCbs;
    kalamosCbs.period = 30;

    kalamosCbs.periodicCallback = std::bind(&Context::tick, &context);
    kalamosCbs.stereoYuvCallback = std::bind(&Context::stereoImgCallback, &context, std::placeholders::_1);

    std::unique_ptr<kalamos::Context> kalamosContext = kalamos::init(kalamosCbs);
    if(kalamosContext!=nullptr){

        context.setKalamosContext(kalamosContext.get()); 
 
        kalamosContext->run();  

    }

    return 0;
}
