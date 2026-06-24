#ifndef RULES_HPP
#define RULES_HPP

#include "boidz.hpp"
 
namespace boidz {

float get_distance(const Boid &b1, const Boid &b2);

bool check_critical_distance(const float &d_s, const Boid &b1,
                             const Boid &b2);

std::vector<Boid>get_neighbours(const float &distance,
                                       const Boid &s, const Flock &f);

Coords separation(const Parameters &par, const std::vector<Boid> &neighbours,
                                  const Boid &a);

Coords alignment(const Parameters &par, const std::vector<Boid> &neighbours,
                                 const Boid &s);

Coords cohesion(const Parameters &par, const std::vector<Boid> &neighbours,
                                const Boid &s);

Coords hunt_the_flock(const Boid &hunter, const Flock &stormo,
            const Parameters &par);

Coords hunt_neighbours(const Boid &hunter, const Flock &stormo,
    const Parameters &par);

Coords hunter_repulsion(const Boid &hunter, const Boid &prey,
                        const Parameters &par);

Coords create_velocity_with_rules(const Parameters &par,
                                                  const Flock &stormo,
                                                  const Boid &b,
                                                  const Boid &hunter);

} // namespace boidz
#endif
