#include "stats.hpp"
#include "rules.hpp"
#include <vector>
#include <numeric>

namespace boidz
{
  std::vector<float> get_vector_of_speeds(Flock const &f)
  {
    std::vector<float> vector_of_speeds{};
    if (f.flock.size() != 0.f)
    {
      for (const auto &b : f.flock)
      {
        float speed = std::sqrt(b.getVelocity().x * b.getVelocity().x +
                                b.getVelocity().y * b.getVelocity().y);
        vector_of_speeds.push_back(speed);
      }
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

  Analytics mean_and_error(const Flock &f, const char &type)
  {
    Analytics result{0.f, 0.f};
    if (type == 'speed')
    {
      auto vector_of_speeds = get_vector_of_speeds(f);
      float mean_speed = std::accumulate(vector_of_speeds.begin(), vector_of_speeds.end(), 0.f) /
                         static_cast<float>(vector_of_speeds.size());
      result.mean = mean_speed;

      std::vector<float> v_o_speeds_squared;
      float s_squared;

      for (auto &s : vector_of_speeds)
      {
        s_squared = s * s;
        v_o_speeds_squared.push_back(s_squared);
      }
      float mean_speed_squared = std::accumulate(v_o_speeds_squared.begin(), v_o_speeds_squared.end(), 0.f) /
                                 static_cast<float>(v_o_speeds_squared.size());

      float stan_dev_sq = mean_speed_squared - mean_speed;
      float error_of_mean = std::sqrt(stan_dev_sq / static_cast<float>(f.flock.size()));
      result.error = error_of_mean;
      return result;
    }
    else if (type == 'distance')
    {
    }
    else
    {
      std::cout << "Error in function call" << '\n';
      return result;
    }
  }

} // namespace boidz
