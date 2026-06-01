#include "rules.hpp"
#include <vector>

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

} // namespace boidz
