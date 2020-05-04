#pragma once

#include <array>
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

#include "o80_roboball2d/item.hpp"

namespace o80_roboball2d
{
class TorquesAction
{
public:
    TorquesAction();
    TorquesAction(double t1, double t2, double t3, bool relative = true);

    void set_torques(double t1, double t2, double t3, bool relative = true);
    bool are_torques_relative() const;
    std::array<double, 3> get_torques() const;

    std::string to_string() const;

    bool is_valid() const;
    
public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(id, torques_, relative_torques_, valid_);
    }

public:
    static int get_id();
    int id;
    
private:
    friend shared_memory::private_serialization;

    // Mode 1 : torque control (for real robot)
    // used to send torques to the robot
    std::array<double, 3> torques_;
    bool torques_set_;
    bool relative_torques_;
    bool valid_;
  
};
}
