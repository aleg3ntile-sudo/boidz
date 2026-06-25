#ifndef STATS_HPP
#define STATS_HPP

#include "boidz.hpp"

namespace boidz
{
    struct Statistics
    {
        std::vector<float> vector_of_mean_speeds{};
        std::vector<float> vector_of_mean_distances{};
        std::vector<float> vector_of_standard_deviations_speed{};
        std::vector<float> vector_of_standard_deviations_distance{};
    };

    struct Analytics
    {
        float mean;
        float error;
    };
    
    std::vector<float> get_vector_of_speeds(Flock const &f);
    float mean_speed(Flock const &f);
    std::vector<float> get_vector_of_distances(Flock const &f);
    float mean_distance(Flock const &f);
    float standard_deviation(std::vector<float> const &v);
    Analytics mean_and_error(const Flock &f, const char &type);

} // namespace boidz

#endif // STATS_HPP
