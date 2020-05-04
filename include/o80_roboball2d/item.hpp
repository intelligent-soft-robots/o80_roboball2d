#pragma once
#include <iostream>
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80_roboball2d
{
class Item
{
public:
    Item() : angle(0), angular_velocity(0), torque(0), desired_torque(0)
    {
        position = {0, 0};
        linear_velocity = {0, 0};
    }
    std::array<double, 2> position;
    double angle;
    std::array<double, 2> linear_velocity;
    double angular_velocity;
    double torque;
    double desired_torque;
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(position,
                angle,
                linear_velocity,
                angular_velocity,
                torque,
                desired_torque);
    }
    void console() const
    {
        std::cout << "\tposition: " << position[0] << " " << position[1] << "\n"
                  << "\tangle: " << angle << "\n"
                  << "\tlinear velocity: " << linear_velocity[0] << " "
                  << linear_velocity[1] << "\n"
                  << "\tangular velocity: " << angular_velocity << "\n"
                  << "\tmeasured torque: " << torque << "\n"
                  << "\tdesired torque: " << desired_torque << "\n";
    }
};
}
