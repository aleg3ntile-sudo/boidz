#include "rules.hpp"
#include <vector>
#include <numeric>

namespace boidz
{

  Coords mean_velocity(Flock const &f)
  {
    Coords mean_velocity{0., 0.};
    if (f.flock.size() != 0)
    {
      for (auto &b : f.flock)
      {
        mean_velocity.x += std::abs(b.getVelocity().x);
        mean_velocity.y += std::abs(b.getVelocity().y);
      }
      mean_velocity.x = (mean_velocity.x / f.flock.size());
      mean_velocity.y = (mean_velocity.y / f.flock.size());
      return mean_velocity;
    }
    else
    {
      return mean_velocity;
    }
  }

  std::vector<float> get_vector_of_distances(Flock const &f)
  {
    std::vector<float> vector_of_distances{};
    if (f.flock.size() != 0)
    {
      for (auto first = f.flock.begin(), last = f.flock.end() - 1; first != last; ++first)
      {
        float boidz_distance = get_distance(*first, *(first + 1));
        vector_of_distances.push_back(boidz_distance);
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
                       vector_of_distances.size());
      return mean_distance;
    }
    else
    {
      return mean_distance;
    }
  }

  Coords centre_of_mass_position(Flock const &f)
  {
    Coords centre_of_mass{0., 0.};
    if (f.flock.size() != 0)
    {
      for (auto &b : f.flock)
      {
        centre_of_mass.x += b.getVelocity().x;
        centre_of_mass.y += b.getVelocity().y;
      }
      centre_of_mass.x = (centre_of_mass.x / f.flock.size());
      centre_of_mass.y = (centre_of_mass.y / f.flock.size());
      return centre_of_mass;
    }
    else
    {
      return centre_of_mass;
    }
  }

  float standard_deviation(std::vector<float> const &v)
  {
    float standard_deviation{0.f};
    if (v.size() != 0)
    {
      float mean = std::accumulate(v.begin(), v.end(), 0.f) / v.size();
      float sum_of_squared_deviations = 0.f;
      for (auto &x : v)
      {
        sum_of_squared_deviations += std::pow(x - mean, 2);
      }
      standard_deviation = std::sqrt(sum_of_squared_deviations / v.size());
      return standard_deviation;
    }
    else
    {
      return standard_deviation;
    }
  }
  float standard_deviation(std::vector<SingleBoid> const &v)
  {
    float standard_deviation{0.f};
    if (v.size() != 0)
    {
      Coords mean{0.f, 0.f};
      for (auto &b : v)
      {
        mean.x += b.getVelocity().x;
        mean.y += b.getVelocity().y;
      }
      mean.x = mean.x / v.size();
      mean.y = mean.y / v.size();

      float sum_of_squared_deviations = 0.f;
      for (auto &b : v)
      {
        sum_of_squared_deviations += std::pow(b.getVelocity().x - mean.x, 2) + std::pow(b.getVelocity().y - mean.y, 2);
      }
      standard_deviation = std::sqrt(sum_of_squared_deviations / v.size());
      return standard_deviation;
    }
    else
    {
      return standard_deviation;
    }
  }

} // namespace boidz
