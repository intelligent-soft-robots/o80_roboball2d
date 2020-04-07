#include "o80/pybind_helper.hpp"


template<class ACTION,class STANDALONE>
void create_python_bindings(pybind11::module &m,std::string standalone_class_name)
{
    o80::create_python_bindings<Driver<BallGunAction>,
                                StandaloneBallGun<QUEUE>,
                                std::string>(m, false, false, true);

}
