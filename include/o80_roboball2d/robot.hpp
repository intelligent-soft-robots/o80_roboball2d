#pragma once

#include "shared_memory/shared_memory.hpp"
#include "shared_memory/serializer.hpp"
#include "o80_roboball2d/item.hpp"

namespace o80_roboball2d
{
class Robot
{
public:
    std::array<Item, 3> joints;
    std::array<Item, 2> rods;
    Item racket;
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(joints, rods, racket);
    }
    void console() const
    {
        std::cout << "robot:\n";
        for (int dof = 0; dof < 3; dof++)
        {
            std::cout << "joint " << dof << ":\n";
            joints[dof].console();
        }
        for (int i = 0; i < 2; i++)
        {
            std::cout << "rod " << i << ":\n";
            rods[i].console();
        }
        std::cout << "racket:\n";
        racket.console();
    }
};
}
