#ifndef RULES_HPP
#define RULES_HPP

#include "boidz.hpp"
#include <cmath>

namespace boidz
{

    bool check_distance(double d_s, SingleBoid b1, SingleBoid b2)
    {
        Object a = b1.getPosition();
        Object b = b2.getPosition();

        if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2))
        {
            return true;
        }
        return false;
    }

    void separation(double s, double d_s, Flock stormo)
    {
        for (SingleBoid x : stormo.flock)
        {
            if (check_distance(d_s, x, ))
        }
    }

}

#endif
