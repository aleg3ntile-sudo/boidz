#include <vector>
#include <algorithm>

#ifndef BOIDZ_HPP
#define BOIDZ_HPP

namespace boidz
{

    Object operator+(Object &p, Object &q)
    {
        Object sum{p.x + q.x, p.y + q.y};
        return sum;
    }

    struct Parameters
    {
        double s;   // parametro di separazione
        double d;   // parametro vicini
        double d_s; // paramtero vicini critici
        double a; //parametro di allineamento
    };
    struct Object
    {
        double x;
        double y;
    };

    class SingleBoid
    {
        Object position;
        Object velocity;
        int cardinality; // e che nessuno osi fiatare

    public:
        auto getCardinality() { return cardinality; }
        auto getPosition() { return position; }
        auto getVelocity() { return velocity; }

        Object update_Position_in_time() {}
        Object update_Velocity_in_time() {}
    };

    class Flock
    {

    public:
        std::vector<SingleBoid> flock{};
    };

}

#endif