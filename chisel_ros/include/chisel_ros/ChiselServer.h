// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CHISELSERVER_H_
#define CHISELSERVER_H_

#include <deque>
#include <Eigen/Eigen>

#include <chisel_msgs/ResetService.h>
#include <chisel_msgs/PauseService.h>
#include <chisel_msgs/SaveMeshService.h>
#include <chisel_msgs/GetAllChunksService.h>
#include <chisel_msgs/UpdateAllMeshService.h>

#include <memory>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/GeneralCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace chisel_ros
{

typedef float DepthData;
typedef uint8_t ColorData;

class ChiselServer
{
  public:
    enum class FusionMode
    {
        DepthImage,
        PointCloud
    };

    struct RosCameraTopic
    {
        std::string imageTopic;
        // std::string infoTopic;
        std::string transform;
        // chisel::PinholeCamera cameraModel;
        ros::Subscriber imageSubscriber;
        // ros::Subscriber infoSubscriber;
        // ros::Publisher lastPosePublisher;
        // ros::Publisher frustumPublisher;
        chisel::Transform lastPose;
        ros::Time lastImageTimestamp;
        bool gotPose;
        // bool gotInfo;
        bool gotImage;

        message_filters::Subscriber<sensor_msgs::Image> *sub_image;
        // message_filters::Subscriber<sensor_msgs::CameraInfo> *sub_info;
    };

    struct RosPointCloudTopic
    {
        std::string cloudTopic;
        std::string transform;
        ros::Subscriber cloudSubscriber;
        chisel::Transform lastPose;
        ros::Time lastTimestamp;
        bool gotPose;
        bool gotCloud;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_point_cloud;
    };

    struct RosPoseStampedTopic
    {
        std::string poseTopic_name;
        message_filters::Subscriber<nav_msgs::Odometry> *sub_pose;
    };

    ChiselServer();
    ChiselServer(const ros::NodeHandle &nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode);
    virtual ~ChiselServer();

    inline chisel::ChiselPtr GetChiselMap()
    {
        return chiselMap;
    }
    inline void SetChiselMap(const chisel::ChiselPtr value)
    {
        chiselMap = value;
    }

    inline const std::string &GetBaseTransform() const
    {
        return baseTransform;
    }
    inline const std::string &GetMeshTopic() const
    {
        return meshTopic;
    }

    void SetupProjectionIntegrator(chisel::TruncatorPtr truncator, uint16_t weight, bool useCarving, float carvingDist);
    void SetupMeshPublisher(const std::string &meshTopic);
    void SetupChunkBoxPublisher(const std::string &boxTopic);
    void SetupDepthPosePublisher(const std::string &depthPoseTopic);
    void SetupColorPosePublisher(const std::string &colorPoseTopic);
    void SetupDepthFrustumPublisher(const std::string &frustumTopic);
    void SetupColorFrustumPublisher(const std::string &frustumTopic);
    void SetupLocalChunksPublisher(const std::string &localChunksTopic);

    void PublishMeshes();
    void PublishChunkBoxes();
    void PublishLatestChunkBoxes();
    void PublishDepthPose();
    void PublishColorPose();
    void PublishDepthFrustum();
    void PublishColorFrustum();
    void PublishLocalChunks();

    void SubscribeDepthImage(const std::string &depthImageTopic, const std::string &cameraInfoTopic, const std::string &transform);
    void DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
    void DepthImageCallback(sensor_msgs::ImageConstPtr depthImage);

    void SubscribeColorImage(const std::string &colorImageTopic, const std::string &cameraInfoTopic, const std::string &transform);
    void ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
    void ColorImageCallback(sensor_msgs::ImageConstPtr colorImage);

    void OdometryCallback(const nav_msgs::OdometryConstPtr msg);

    void Subscribe_image_All(
        const std::string &depth_imageTopic,
        const std::string &color_imageTopic,
        const std::string &odometry_topic);
    void Callback_image_All(
        sensor_msgs::ImageConstPtr depth_image,
        sensor_msgs::ImageConstPtr color_image,
        nav_msgs::OdometryConstPtr msg);

    void Subscribe_pointcloud_All(  const std::string &point_cloud_topic, const std::string &odometry_topic,
                                    const bool pointcloud_transformed);
    void Callback_pointcloud_All(   sensor_msgs::PointCloud2ConstPtr point_cloud, nav_msgs::OdometryConstPtr msg);

    void SubscribePointCloud(const std::string &topic);
    void PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud);

    void IntegrateLastDepthImage();
    void IntegrateLastPointCloud();
    void FillMarkerTopicWithMeshes(visualization_msgs::Marker *marker, visualization_msgs::Marker *marker2);
    inline void SetBaseTransform(const std::string &frameName)
    {
        baseTransform = frameName;
    }

    inline bool HasNewData()
    {
        return hasNewData;
    }

    inline float GetNearPlaneDist() const
    {
        return nearPlaneDist;
    }
    inline float GetFarPlaneDist() const
    {
        return farPlaneDist;
    }
    inline void SetNearPlaneDist(float dist)
    {
        nearPlaneDist = dist;
    }
    inline void SetFarPlaneDist(float dist)
    {
        farPlaneDist = dist;
    }
    inline void SetLocalChunksSetSize(int size_x,int size_y,int size_z)
    {
        localChunksSize_x=size_x;
        localChunksSize_y=size_y;
        localChunksSize_z=size_z;
    }
    bool Reset(chisel_msgs::ResetService::Request &request, chisel_msgs::ResetService::Response &response);
    bool TogglePaused(chisel_msgs::PauseService::Request &request, chisel_msgs::PauseService::Response &response);
    bool SaveMesh(chisel_msgs::SaveMeshService::Request &request, chisel_msgs::SaveMeshService::Response &response);
    bool GetAllChunks(chisel_msgs::GetAllChunksService::Request &request, chisel_msgs::GetAllChunksService::Response &response);
    bool Update_all_mesh(chisel_msgs::UpdateAllMeshService::Request &request, chisel_msgs::UpdateAllMeshService::Response &response);

    inline bool IsPaused()
    {
        return isPaused;
    }
    inline void SetPaused(bool paused)
    {
        isPaused = paused;
    }

    void AdvertiseServices();

    inline FusionMode GetMode()
    {
        return mode;
    }
    inline void SetMode(const FusionMode &m)
    {
        mode = m;
    }

    void SetDepthImage(const sensor_msgs::ImageConstPtr &img);
    void SetDepthPose(const Eigen::Affine3f &tf);
    void SetColorImage(const sensor_msgs::ImageConstPtr &img);
    void SetColorPose(const Eigen::Affine3f &tf);
    void SetColorCameraInfo(const sensor_msgs::CameraInfoConstPtr &info);
    void SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr &info);
    void setPinholeCameraType(const float fx, const float fy, const float cx, const float cy, const int width, const int height);
    void setFisheyeCameraType(const int degree_step, const int width, const int height);
    void setPinholeCameraType(
    const float fx, const float fy,
    const float cx, const float cy,
    const int width, const int height,
    const float farPlaneDist, const float nearPlaneDist);
    void setFisheyeCameraType(
    const int degree_step, const int width, const int height,
    const float farPlaneDist);

  protected:
    visualization_msgs::Marker CreateFrustumMarker(const chisel::Frustum &frustum);

    ros::NodeHandle nh;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy_for_image;
    message_filters::Synchronizer<MySyncPolicy_for_image> *sync_image;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy_for_pointcloud;
    message_filters::Synchronizer<MySyncPolicy_for_pointcloud> *sync_pointcloud;

    chisel::ChiselPtr chiselMap;
    // tf::TransformListener transformListener;
    std::shared_ptr<chisel::DepthImage<DepthData>> lastDepthImage;
    std::shared_ptr<chisel::ColorImage<ColorData>> lastColorImage;
    chisel::PointCloudPtr lastPointCloud;
    std::vector<float> cloud_certianity;
    chisel::ProjectionIntegrator projectionIntegrator;
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    std::string localChunksTopic;

    ros::Publisher meshPublisher;
    ros::Publisher chunkBoxPublisher;
    ros::Publisher latestChunkPublisher;
    ros::Publisher localChunksPublisher;
    ros::ServiceServer resetServer;
    ros::ServiceServer pauseServer;
    ros::ServiceServer saveMeshServer;
    ros::ServiceServer getAllChunksServer;
    ros::ServiceServer update_all_mesh;
    RosCameraTopic depthCamera;
    RosCameraTopic colorCamera;
    chisel::Transform CameraPose;
    chisel::GeneralCamera general_camera;
    chisel::PinholeCamera to_delete_temp;
    RosPointCloudTopic pointcloudTopic;
    RosPoseStampedTopic poseTopic;
    bool useColor;
    bool hasNewData;
    float nearPlaneDist;
    float farPlaneDist;
    int localChunksSize_x;
    int localChunksSize_y;
    int localChunksSize_z;
    bool isPaused;
    bool pointcloud_transformed;
    FusionMode mode;
};
typedef std::shared_ptr<ChiselServer> ChiselServerPtr;
typedef std::shared_ptr<const ChiselServer> ChiselServerConstPtr;

} // namespace chisel

#endif // CHISELSERVER_H_
