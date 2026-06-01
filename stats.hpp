#ifndef STATS_HPP
#define STATS_HPP

#include "boidz.hpp"

namespace boidz {

Coords mean_velocity(Flock const &f);
float mean_speed(Flock const &f);
std::vector<float> get_vector_of_speeds(Flock const &f);
std::vector<float> get_vector_of_distances(Flock const &f);
float mean_distance(Flock const &f);
Coords centre_of_mass_position(Flock const &f);
float standard_deviation(std::vector<float> const &v);
float standard_deviation(std::vector<SingleBoid> const &v);

} // namespace boidz

#endif // STATS_HPP
