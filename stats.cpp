#include "stats.hpp"
#include "rules.hpp"
#include <vector>
#include <numeric>

namespace boidz
{
  std::vector<float> get_vector_of_speeds(Flock const &f)
  {
    std::vector<float> vector_of_speeds{};
    if (f.flock.size() != 0)
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

  Coords mean_and_error(const Flock &f, const std::string &type)
  {
    Coords result{0.f, 0.f};
    std::vector<float> vector_of{};
    if (type == "speed")
    {
      vector_of = get_vector_of_speeds(f);
    }
    else if (type == "distance")
    {
      vector_of = get_vector_of_distances(f);
    }
    else
    {
      std::cout << "Error in function call" << '\n';
      return result = {-1.f, -1.f};
    }

    float mean = std::accumulate(vector_of.begin(), vector_of.end(), 0.f) /
                 static_cast<float>(vector_of.size());
    result.x = mean;

    std::vector<float> v_of_squares;
    float square;

    for (auto &s : vector_of)
    {
      square = s * s;
      v_of_squares.push_back(square);
    }
    float mean_squared = std::accumulate(v_of_squares.begin(), v_of_squares.end(), 0.f) /
                         static_cast<float>(v_of_squares.size());

    float stan_dev_sq = mean_squared - mean;
    float error_of_mean = std::sqrt(stan_dev_sq / static_cast<float>(f.flock.size()));
    result.y = error_of_mean;
    return result;
  }

  void remove_overflow(std::vector<float> &v, const size_t &max)
  {
    if (v.size() > max)
    {
      v.erase(v.begin(), v.begin() + static_cast<long int>((v.size() - max)));
    }
  }

} // namespace boidz
