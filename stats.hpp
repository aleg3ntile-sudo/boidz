#ifndef STATS_HPP
#define STATS_HPP

#include "boidz.hpp"

namespace boidz
{
    struct Statistics
    {
        std::vector<float> vector_of_mean_speeds{};
        std::vector<float> vector_of_mean_distances{};
        std::vector<float> vector_of_errors_speed{};
        std::vector<float> vector_of_errors_distance{};
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
    Coords mean_and_error(const Flock &f, const std::string &type);
    void remove_overflow(std::vector<float> &v, const size_t &max);

} // namespace boidz

#endif // STATS_HPP
