#include <vector>
#include <algorithm>
#include "rules.hpp"

#ifndef BOIDZ_HPP
#define BOIDZ_HPP

namespace boidz
{
    struct Coords
    {
        double x;
        double y;
    };

    Coords operator+(const Coords &p, const Coords &q)
    {
        return {p.x + q.x, p.y + q.y};
    }

    struct Parameters
    {
        double s;   // parametro di separazione
        double d;   // parametro vicini
        double d_s; // paramtero vicini critici
        double a;   // parametro di allineamento
        double c;   // parametro di coesione
    };
    class Flock;
    class SingleBoid
    {
        Coords position;
        Coords velocity;
        int cardinality; // e che nessuno osi fiatare

    public:
        SingleBoid() : position{}, velocity{}, cardinality(0) {}
        SingleBoid(Coords p, Coords v, int i) : position(p), velocity(v), cardinality(i) {}
        auto getCardinality() { return cardinality; }
        auto getPosition() { return position; }
        auto getVelocity() { return velocity; }

        void update_Position_in_time_for_single_boid(double time)
        {
            position.x = position.x + time * velocity.x;
            position.y = position.y + time * velocity.y;
        }

        void update_Velocity_with_rules_for_single_boid(Coords updated_velocity)
        {
           velocity = velocity + updated_velocity;
        }
        // Coords update_Velocity_in_time_for_single_boid() {}
    };

    class Flock
    {

    public:
        std::vector<SingleBoid> flock{};
        Flock() : flock{} {}
        Flock(int N) : flock(N) {}
    };

}

#endif