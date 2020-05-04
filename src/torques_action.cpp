#include "o80_roboball2d/torques_action.hpp"

namespace o80_roboball2d
{
    
    TorquesAction::TorquesAction()
	: id(TorquesAction::get_id()), relative_torques_(true),valid_(false)
    {
	set_torques(0, 0, 0, true);
    }

    TorquesAction::TorquesAction(double t1, double t2, double t3, bool relative)
	: valid_(true)
    {
	set_torques(t1, t2, t3, relative);
    }

    int TorquesAction::get_id()
    {
	static int id;
	id++;
	return id;
    }

    std::string TorquesAction::to_string() const
    {
	return "action " + std::to_string(id) + " | " +
	    std::to_string(torques_[0]) + " | " + std::to_string(torques_[1]) +
	    " | " + std::to_string(torques_[2]);
    }

    bool TorquesAction::is_valid() const
    {
	return valid_;
    }
    
    void TorquesAction::set_torques(double t1, double t2, double t3, bool relative)
    {
	torques_[0] = t1;
	torques_[1] = t2;
	torques_[2] = t3;
	torques_set_ = true;
	if (relative)
	    {
		relative_torques_ = true;
	    }
	valid_=true;
    }

    bool TorquesAction::are_torques_relative() const
    {
	return relative_torques_;
    }

    std::array<double, 3> TorquesAction::get_torques() const
    {
	return torques_;
    }
    
}
