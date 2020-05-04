#pragma once

#include "o80_roboball2d/robot.hpp"


namespace o80_roboball2d
{

    template<int NB_ROBOTS,int NB_BALLS>
    class WorldState
    {
    public:
	WorldState()
	    : id(WorldState::get_id()),
	      ball_hits_floor_position(0),
	      ball_hits_floor(false),
	      ball_hits_racket(false),
	      valid(false),
	      t(0)
	{}
	int id;
	Item ball;
	std::array<Item,NB_BALLS> balls;
	Robot robot;
	std::array<Item,NB_ROBOTS> robots;
	double ball_hits_floor_position;
	bool ball_hits_floor;
	bool ball_hits_racket;
	std::array<double,NB_BALLS> balls_hits_floor_position;
	std::array<bool,NB_BALLS> balls_hits_floor;
	std::array<bool,NB_BALLS> balls_hits_racket;
	bool valid;
	double t;
	template <class Archive>
	void serialize(Archive& archive)
	{
	    archive(id,
		    ball,
		    robot,
		    balls,
		    robots,
		    ball_hits_floor_position,
		    ball_hits_floor,
		    ball_hits_racket,
		    balls_hits_floor_position,
		    balls_hits_floor,
		    balls_hits_racket,
		    valid,
		    t);
	}
	static int get_id()
	{
	    static int id = 0;
	    id++;
	    return id;
	}
	void console(bool print_robot=false, bool print_ball=false) const
	{
	    std::cout << "\nWorld State " << id << " (" << valid << ") "
		      << " (" << t << ") \n";
	    std::cout << "nb robots: " << robots.size() << "\n";
	    std::cout << "nb balls: " << balls.size() << "\n";
	    if (print_ball)
		{
		    std::cout << "Ball:\n";
		    ball.console();
		    std::cout << "ball hits floor: " << ball_hits_floor << "\n"
			      << "ball_hits_floor_position: "
			      << ball_hits_floor_position << "\n"
			      << "ball hits racket: " << ball_hits_racket << "\n";
		}
	    if (print_robot)
		{
		    robot.console();
		}
	}
    };

    typedef WorldState<1,1> OneBallWorldState;
    typedef WorldState<1,5> FiveBallsWorldState;
}
