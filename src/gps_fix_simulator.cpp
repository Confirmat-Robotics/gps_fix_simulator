// File: src/gps_fix_simulator.cpp

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Helpers.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <cmath>
#include <unordered_set>

namespace gps_fix_simulator
{
class GPSFixSimulatorPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPostUpdate,
                               public gz::sim::ISystemHandle
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override
  {
    this->worldEntity = entity;
    this->modelName = sdf->Get<std::string>("model_name", "bigbot").first;
    this->linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->childFrame = sdf->Get<std::string>("child_frame", "TF_POI").first;
    this->parentFrame = sdf->Get<std::string>("parent_frame", "TF_ECEF").first;
    this->enuFrame = sdf->Get<std::string>("enu_frame", "FP_ENU").first;

    // Initialize ROS2 if not already initialized
    this->ros_context = std::make_shared<rclcpp::Context>();
    this->ros_context->init(0, nullptr);

    rclcpp::NodeOptions options;
    options.context(this->ros_context);
    this->node = std::make_shared<rclcpp::Node>("gps_fix_simulator_node", options);

    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);

    // Spin the ROS node in a background thread
    this->ros_thread = std::thread([this]() {
      rclcpp::spin(this->node);
    });

    std::cout << "[GPSFixSimulatorPlugin] Configured for model: " << this->modelName 
              << ", link: " << this->linkName << std::endl;

    // Subscribe to EntityCreated event
    eventMgr.Subscribe<gz::sim::events::EntityCreated>(
      [this](const gz::sim::events::EntityCreated &event, gz::sim::EntityComponentManager &ecm)
      {
        auto nameComp = ecm.Component<gz::sim::components::Name>(event.Entity());
        if (nameComp)
        {
          std::cout << "[GPSFixSimulatorPlugin] Entity created: " << nameComp->Data() << std::endl;
          this->newEntities.insert(event.Entity());
        }
      });
  }

  ~GPSFixSimulatorPlugin()
  {
    if (this->node)
    {
      rclcpp::shutdown(this->ros_context);
    }
    if (this->ros_thread.joinable())
    {
      this->ros_thread.join();
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    // Skip if ROS is not properly initialized
    if (!this->node || !rclcpp::ok(this->ros_context))
    {
      return;
    }

    // Process newly created entities
    for (auto entity : this->newEntities)
    {
      auto nameComp = ecm.Component<gz::sim::components::Name>(entity);
      if (nameComp)
      {
        std::cout << "[GPSFixSimulatorPlugin] Processing new entity: " << nameComp->Data() << std::endl;

        // Check if the entity matches the target model
        if (nameComp->Data() == this->modelName)
        {
          this->modelEntity = entity;
          std::cout << "[GPSFixSimulatorPlugin] Found target model: " << this->modelName << std::endl;
        }
      }
    }
    this->newEntities.clear();

    // Debug: List all models in the world if we haven't found our target model
    auto allModels = ecm.EntitiesByComponents(gz::sim::components::Model());
    if (allModels.size() > 0)
    {
      if (allModels.size() != this->lastModelCount) {
        this->lastModelCount = allModels.size();
      }
      std::cout << "[GPSFixSimulatorPlugin] DEBUG: All models in world (count: " << allModels.size() << "):" << std::endl;
      for (auto modelEnt : allModels)
      {
        auto nameComp = ecm.Component<gz::sim::components::Name>(modelEnt);
        if (nameComp)
        {
          std::cout << "  Model: " << nameComp->Data() << std::endl;
          
          // List all links in this model
          auto linkEntities = ecm.ChildrenByComponents(modelEnt, gz::sim::components::Link());
          std::cout << "    Links: ";
          for (auto linkEnt : linkEntities)
          {
            auto linkNameComp = ecm.Component<gz::sim::components::Name>(linkEnt);
            if (linkNameComp) std::cout << linkNameComp->Data() << " ";
          }
          std::cout << std::endl;
        }
      }
    }

    // Find model by name if not found yet
    if (this->modelEntity == gz::sim::kNullEntity)
    {
      this->modelEntity = ecm.EntityByComponents(
        gz::sim::components::Name(this->modelName),
        gz::sim::components::Model());
      if (this->modelEntity == gz::sim::kNullEntity)
        return; // Model not found yet
      
      std::cout << "[GPSFixSimulatorPlugin] Found model: " << this->modelName << std::endl;
    }

    // Find link in model if not found yet
    if (this->linkEntity == gz::sim::kNullEntity)
    {
      auto linkEntities = ecm.ChildrenByComponents(this->modelEntity, gz::sim::components::Link());
      for (auto linkEnt : linkEntities)
      {
        auto nameComp = ecm.Component<gz::sim::components::Name>(linkEnt);
        if (nameComp && nameComp->Data() == this->linkName)
        {
          this->linkEntity = linkEnt;
          std::cout << "[GPSFixSimulatorPlugin] Found link: " << this->linkName << std::endl;
          break;
        }
      }
      if (this->linkEntity == gz::sim::kNullEntity)
      {
        // Debug: List available links
        std::cout << "[GPSFixSimulatorPlugin] Available links in model " << this->modelName << ": ";
        for (auto linkEnt : linkEntities)
        {
          auto nameComp = ecm.Component<gz::sim::components::Name>(linkEnt);
          if (nameComp) std::cout << nameComp->Data() << " ";
        }
        std::cout << std::endl;
        return; // Link not found yet
      }
    }

    // Check if pose component exists
    auto poseComp = ecm.Component<gz::sim::components::WorldPose>(this->linkEntity);
    if (!poseComp) return;
    const auto &pose = poseComp->Data();

    // Check for spherical coordinates
    auto sphericalComp = ecm.Component<gz::sim::components::SphericalCoordinates>(this->worldEntity);
    if (!sphericalComp) 
    {
      std::cerr << "[GPSFixSimulatorPlugin] No spherical coordinates found in world" << std::endl;
      return;
    }
    auto spherical = sphericalComp->Data();

    // Convert local position to lat/lon/alt
    auto latLonAlt = spherical.SphericalFromLocalPosition(pose.Pos());
    
    // Validate coordinates
    if (std::isnan(latLonAlt.X()) || std::isnan(latLonAlt.Y()) || std::isnan(latLonAlt.Z()))
    {
      std::cerr << "[GPSFixSimulatorPlugin] Invalid coordinates calculated" << std::endl;
      return;
    }

    constexpr double degToRad = M_PI / 180.0;
    double lat_rad = latLonAlt.X() * degToRad;
    double lon_rad = latLonAlt.Y() * degToRad;
    double alt = latLonAlt.Z();

    constexpr double a = 6378137.0;
    constexpr double e_sq = 6.69437999014e-3;
    double N = a / std::sqrt(1 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));

    double x = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    double y = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    double z = ((1 - e_sq) * N + alt) * std::sin(lat_rad);

    // Validate ECEF coordinates
    if (std::isnan(x) || std::isnan(y) || std::isnan(z))
    {
      std::cerr << "[GPSFixSimulatorPlugin] Invalid ECEF coordinates" << std::endl;
      return;
    }

    // Publish transforms
    try
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->node->get_clock()->now();
      tf_msg.header.frame_id = this->parentFrame;
      tf_msg.child_frame_id = this->childFrame;
      tf_msg.transform.translation.x = x;
      tf_msg.transform.translation.y = y;
      tf_msg.transform.translation.z = z;
      tf_msg.transform.rotation.x = pose.Rot().X();
      tf_msg.transform.rotation.y = pose.Rot().Y();
      tf_msg.transform.rotation.z = pose.Rot().Z();
      tf_msg.transform.rotation.w = pose.Rot().W();
      this->tf_broadcaster->sendTransform(tf_msg);

      // Publish static ENU transform once
      if (!this->enuPublished)
      {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = tf_msg.header.stamp;
        static_tf.header.frame_id = this->parentFrame;
        static_tf.child_frame_id = this->enuFrame;
        static_tf.transform.translation.x = 0;
        static_tf.transform.translation.y = 0;
        static_tf.transform.translation.z = 0;
        static_tf.transform.rotation.x = 0;
        static_tf.transform.rotation.y = 0;
        static_tf.transform.rotation.z = 0;
        static_tf.transform.rotation.w = 1;
        this->tf_broadcaster->sendTransform(static_tf);
        this->enuPublished = true;
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "[GPSFixSimulatorPlugin] Failed to publish transform: " << e.what() << std::endl;
    }
  }

private:
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  std::string modelName;
  std::string linkName;
  std::string parentFrame;
  std::string childFrame;
  std::string enuFrame;

  std::shared_ptr<rclcpp::Context> ros_context;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::thread ros_thread;
  bool enuPublished = false;
  size_t lastModelCount = 0;

  std::unordered_set<gz::sim::Entity> newEntities;
};
} // namespace gps_fix_simulator

GZ_ADD_PLUGIN(
  gps_fix_simulator::GPSFixSimulatorPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate,
  gz::sim::ISystemHandle)

GZ_ADD_PLUGIN_ALIAS(gps_fix_simulator::GPSFixSimulatorPlugin, "gps_fix_simulator")

