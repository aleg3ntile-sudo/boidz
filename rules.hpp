#ifndef RULES_HPP
#define RULES_HPP

#include "boidz.hpp"
 
namespace boidz {

float get_distance(const SingleBoid &b1, const SingleBoid &b2);

bool check_critical_distance(const float &d_s, const SingleBoid &b1,
                             const SingleBoid &b2);

bool check_neighbours(const float &d, const SingleBoid &b1,
                      const SingleBoid &b2);

std::vector<SingleBoid>get_neighbours(const float &distance,
                                       const SingleBoid &s, const Flock &f);

Coords separation(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                                  const SingleBoid &a);

Coords alignment(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                                 const SingleBoid &s);

Coords cohesion(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                                const SingleBoid &s);

Coords hunt_the_flock(const SingleBoid &hunter, const Flock &stormo,
            const Parameters &par);

Coords hunt_neighbours(const SingleBoid &hunter, const Flock &stormo,
    const Parameters &par);

Coords hunter_repulsion(const SingleBoid &hunter, const SingleBoid &prey,
                        const Parameters &par);

Coords create_velocity_with_rules(const Parameters &par,
                                                  const Flock &stormo,
                                                  const SingleBoid &b,
                                                  const SingleBoid &hunter);

} // namespace boidz
#endif
