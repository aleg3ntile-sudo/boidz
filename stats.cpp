#include "stats.hpp"
#include "rules.hpp"
#include <vector>
#include <numeric>

namespace boidz
{
  std::vector<float> get_vector_of_speeds(Flock const &f)
  {
    std::vector<float> vector_of_speeds{};
    for (const auto &b : f.flock)
    {
      float speed = std::sqrt(b.getVelocity().x * b.getVelocity().x +
                              b.getVelocity().y * b.getVelocity().y);
      vector_of_speeds.push_back(speed);
    }
    return vector_of_speeds;
  }

  float mean_speed(Flock const &f)
  {
    float mean_speed{0.f};
    if (f.flock.size() != 0)
    {
      auto vector_of_speeds = get_vector_of_speeds(f);
      mean_speed = std::accumulate(vector_of_speeds.begin(), vector_of_speeds.end(), 0.f) / static_cast<float>(vector_of_speeds.size());
    }
    return mean_speed;
  }

  std::vector<float> get_vector_of_distances(Flock const &f)
  {
    std::vector<float> vector_of_distances{};
    float boidz_distance{0.f};
    if (f.flock.size() != 0)
    {
      for (const auto &b : f.flock)
      {
        for (const auto &c : f.flock)
        {
          if (b.getCardinality() < c.getCardinality())
          {
            boidz_distance = get_distance(b, c);
            vector_of_distances.push_back(boidz_distance);
          }
        }
      }
      return vector_of_distances;
    }
    else
    {
      return vector_of_distances;
    }
  }

  float mean_distance(Flock const &f)
  {
    float mean_distance{0.};
    if (f.flock.size() != 0)
    {
      auto vector_of_distances = get_vector_of_distances(f);
      mean_distance = (std::accumulate(vector_of_distances.begin(), vector_of_distances.end(), 0.f) /
                       static_cast<float>(vector_of_distances.size()));
      return mean_distance;
    }
    else
    {
      return mean_distance;
    }
  }

  float standard_deviation(std::vector<float> const &v)
  {
    float standard_deviation{0.f};
    if (v.size() != 0)
    {
      float mean = std::accumulate(v.begin(), v.end(), 0.f) / static_cast<float>(v.size());
      float sum_of_squared_deviations = 0.f;
      for (const auto &x : v)
      {
        sum_of_squared_deviations += static_cast<float>(std::pow(x - mean, 2));
      }
      standard_deviation = std::sqrt(sum_of_squared_deviations / static_cast<float>(v.size()));
      return standard_deviation;
    }
    else
    {
      return standard_deviation;
    }
  }
  

} // namespace boidz
