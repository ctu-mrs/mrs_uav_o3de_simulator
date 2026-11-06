#pragma once

#include <Multirotor/MultirotorTypeIds.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace Multirotor
{
    //! Ground truth odometry in the simulation's world frame, publishing nav_msgs/Odometry with the twist in the body
    //! frame. It replaces ROS2OdometrySensorComponent from the ROS2Sensors Gem, whose pose is relative to the body's
    //! first pose and so always starts at zero. The sensor configuration has the same format.
    class WorldOdometrySensorComponent : public ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>
    {
    public:
        using SensorBaseType = ROS2::ROS2SensorComponentBase<ROS2::PhysicsBasedSource>;

        AZ_COMPONENT(WorldOdometrySensorComponent, WorldOdometrySensorComponentTypeId, SensorBaseType);

        WorldOdometrySensorComponent();

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        void Publish();

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;
        nav_msgs::msg::Odometry m_message;
    };
} // namespace Multirotor
