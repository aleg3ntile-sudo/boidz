#ifndef STATS_HPP
#define STATS_HPP

#include "boidz.hpp"

namespace boidz {

Coords mean_velocity(Flock const &f);
Coords centre_of_mass_position(Flock const &f);

} // namespace boidz

#endif // STATS_HPP
