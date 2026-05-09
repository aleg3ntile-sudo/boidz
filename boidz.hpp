#include <vector>
#include <algorithm>

#ifndef BOIDZ_HPP
#define BOIDZ_HPP

namespace boidz
{
    struct Parameters
    {
        double s;   // parametro di separazione
        double d;   // parametro vicini
        double d_s; // paramtero vicini critici
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

        Object updatePosition() {}
        Object updateVelocity() {}
    };

    class Flock
    {

    public:
        std::vector<SingleBoid> flock{};
    };

}

#endif