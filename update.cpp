#include "boidz.hpp"
#include "rules.hpp"

using namespace boidz;

Coords update_Velocity_with_rules_for_single_boid(Parameters par, Flock stormo, SingleBoid b)
{
    auto new_velocity = b.getVelocity();
    new_velocity = new_velocity + separation_for_single_boid(par, stormo, b) +
    alignment_for_single_boid(par, stormo, b) + cohesion_for_single_boid(par, stormo, b);
    return new_velocity;

}