#ifndef RULES_HPP
#define RULES_HPP

#include "boidz.hpp"
#include <cmath>

namespace boidz
{
    double get_distance(SingleBoid b1, SingleBoid b2)
    {
        Object a = b1.getPosition();
        Object b = b2.getPosition();

        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
    bool check_critic_distance(double d_s, SingleBoid b1, SingleBoid b2)
    {
        Object a = b1.getPosition();
        Object b = b2.getPosition();

        if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2))
        {
            return true;
        }
        return false;
    }

    bool check_neighbours(double d, SingleBoid b1, SingleBoid b2)
    {
        Object a = b1.getPosition();
        Object b = b2.getPosition();

        if (std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d, 2))
        {
            return true;
        }
        return false;
    }

    std::vector<SingleBoid> get_neighbours(double distance, SingleBoid s, Flock f)
    {
        std::vector<SingleBoid> neighbours{};
        for (auto x : f.flock)
        {
            if (check_neighbours(distance, s, x) == true)
            {
                if (s.getCardinality() != x.getCardinality())
                {
                    neighbours.push_back(x);
                }
            }
        }
        return neighbours;
    }

    Object separation_for_single_boid(Parameters const &par, Flock &stormo, SingleBoid &a)
    {
        double sum_x{0};
        double sum_y{0};
        auto neighbours = get_neighbours(par.d, a, stormo);
        for (auto b : neighbours)
        {
            if (check_critic_distance(par.d_s, a, b))
            {
                sum_x += a.getPosition().x - b.getPosition().x;
                sum_y += a.getPosition().y - b.getPosition().y;
            }
        }

        Object v_separation{-par.s * sum_x, -par.s * sum_y};
        return v_separation;
    }

}

#endif
