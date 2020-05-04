#include "o80_roboball2d/ball_gun_action.hpp"

namespace o80_roboball2d
{
    
    BallGunAction::BallGunAction() : id(BallGunAction::get_id()), shoot_(false), valid_(false)
    {
    }

    BallGunAction::BallGunAction(bool shoot)
	: id(BallGunAction::get_id()), shoot_(shoot),valid_(true)
    {
    }

    bool BallGunAction::is_valid() const
    {
	return valid_;
    }
    
    int BallGunAction::get_id()
    {
	static int id;
	id++;
	return id;
    }

    std::string BallGunAction::to_string() const
    {
	return "ball gun action " + std::to_string(shoot_) + "\n";
    }

    void BallGunAction::set(bool shoot)
    {
	shoot_ = shoot;
	valid_ = true;
    }

    bool BallGunAction::get() const
    {
	return shoot_;
    }

    
    bool BallGunAction::should_shoot() const
    {
	return shoot_;
    }
    
}
