#pragma once

#include <algorithm>
#include <array>
#include "o80_roboball2d/item.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80_roboball2d
{
class MirrorAction
{
public:
    MirrorAction();
    MirrorAction(const std::array<double, 2>& joint1,
                 const std::array<double, 2>& joint2,
                 const std::array<double, 2>& joint3);
    void set_mirroring_information(const std::array<double, 2>& joint1,
                                   const std::array<double, 2>& joint2,
                                   const std::array<double, 2>& joint3);
    std::array<std::array<double, 2>, 3> get_mirroring_information() const;
    std::string to_string() const;
    bool is_valid() const;
    
public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(id, mirroring_information_,valid_);
    }

public:
    static int get_id();
    int id;

private:
    friend shared_memory::private_serialization;
    std::array<std::array<double, 2>, 3> mirroring_information_;
    bool valid_;
};
}
