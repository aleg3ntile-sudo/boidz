#include "rules.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace boidz
{

  float get_distance(const SingleBoid &b1, const SingleBoid &b2)
  {
    Coords a = b1.getPosition();
    Coords b = b2.getPosition();
    assert(std::isfinite(a.x) && std::isfinite(a.y));
    assert(std::isfinite(b.x) && std::isfinite(b.y));

    float distance = static_cast<float>(std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)));

    assert(distance >= 0.);
    return distance;
  }

  bool check_critical_distance(const float &d_s, const SingleBoid &b1,
                               const SingleBoid &b2)
  {
    assert(d_s >= 0.);
    Coords a = b1.getPosition();
    Coords b = b2.getPosition();
    float critical_distance =
        std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) <= std::pow(d_s, 2);

    assert(critical_distance >= 0.);
    return critical_distance;
  }

  bool check_neighbours(const float &d, const SingleBoid &b1,
                        const SingleBoid &b2)
  {
    Coords a = b1.getPosition();
    Coords b = b2.getPosition();
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy <= d * d;
  }

  std::vector<SingleBoid> get_neighbours(const float &distance,
                                         const SingleBoid &s, const Flock &f)
  {
    assert(distance >= 0.);
    std::vector<SingleBoid> neighbours{};

    for (const auto &x : f.flock)
    {
      if (s.getCardinality() != x.getCardinality() &&
          check_neighbours(distance, s, x))
      {
        neighbours.push_back(x);
      }
    }
    assert(neighbours.size() <= f.flock.size());
    return neighbours;
  }

  Coords separation(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                    const SingleBoid &a)
  {
    float sum_x{0};
    float sum_y{0};
    assert(par.d >= 0.);

    for (
      const auto &b : neighbours)
    {
      if (check_critical_distance(par.d_s, a, b))
      {
        sum_x += b.getPosition().x - a.getPosition().x;
        sum_y += b.getPosition().y - a.getPosition().y;
      }
    }

    Coords v_separation{-(par.s * sum_x), -(par.s * sum_y)};
    assert(par.s >= 0.);
    assert(std::isfinite(v_separation.x) && std::isfinite(v_separation.y));
    return v_separation;
  }

  Coords alignment(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                   const SingleBoid &s)
  {
    Coords v_alignment;
    Coords v_sum{0., 0.};
    auto n = neighbours.size();

    for (const auto &b : neighbours)
    {
      v_sum.x += b.getVelocity().x;
      v_sum.y += b.getVelocity().y;
    }
    if (neighbours.size() >= 1)
    {
      v_alignment = Coords{par.a * ((v_sum.x / static_cast<float>(n)) - s.getVelocity().x),
                           par.a * ((v_sum.y / static_cast<float>(n)) - s.getVelocity().y)};
      assert(par.a >= 0.);
      assert(std::isfinite(v_alignment.x) && std::isfinite(v_alignment.y));
      return v_alignment;
    }
    else
    {
      v_alignment = Coords{0.0, 0.0};
      return v_alignment;
    }
  }

  Coords cohesion(const Parameters &par, const std::vector<SingleBoid> &neighbours,
                  const SingleBoid &s)
  {
    Coords com{0., 0.};
    Coords v_cohesion;
    auto n = neighbours.size();
    for (const auto &b : neighbours)
    {
      com.x += b.getPosition().x / static_cast<float>(n);
      com.y += b.getPosition().y / static_cast<float>(n);
    }

    if (neighbours.size() >= 1)
    {
      v_cohesion = Coords{par.c * (com.x - s.getPosition().x),
                          par.c * (com.y - s.getPosition().y)};
      assert(par.c >= 0.);
      assert(std::isfinite(v_cohesion.x) && std::isfinite(v_cohesion.y));
      return v_cohesion;
    }
    else
    {
      v_cohesion = Coords{0.0, 0.0};
      return v_cohesion;
    }
  }

  Coords hunt_the_flock(const SingleBoid &hunter, const Flock &stormo,
                        const Parameters &par)
  {
    Coords com{0., 0.};
    Coords v_hunt;
    auto s = static_cast<float>(stormo.flock.size());
    for (const auto &b : stormo.flock)
    {
      com.x += b.getPosition().x / s;
      com.y += b.getPosition().y / s;
    }
    if (stormo.flock.size() > 0)
    {
      v_hunt = Coords{par.h * (com.x - hunter.getPosition().x),
                      par.h * (com.y - hunter.getPosition().y)};
      assert(par.h >= 0.);
      assert(std::isfinite(v_hunt.x) && std::isfinite(v_hunt.y));
    }
    else
    {
      v_hunt = Coords{0., 0.};
    }
    return v_hunt;
  }

  Coords hunt_neighbours(const SingleBoid &hunter, const Flock &stormo,
                         const Parameters &par)
  {
    Coords v_chase_neighbours{};
    for (const auto &b : stormo.flock)
    {
      if (check_neighbours(50., hunter, b))
      {
        v_chase_neighbours =
            Coords{par.h * (b.getPosition().x - hunter.getPosition().x),
                   par.h * (b.getPosition().y - hunter.getPosition().y)};
        return v_chase_neighbours;
      }
    }
    return v_chase_neighbours;
  }

  Coords hunter_repulsion(const SingleBoid &hunter, const SingleBoid &prey,
                          const Parameters &par)
  {
    Coords v_prey;
    if (check_critical_distance(par.d, prey, hunter))
    {
      v_prey = Coords{-(par.p * (hunter.getPosition().x - prey.getPosition().x)),
                      -(par.p * (hunter.getPosition().y - prey.getPosition().y))};
      assert(par.p >= 0.);
      assert(std::isfinite(v_prey.x) && std::isfinite(v_prey.y));
    }
    else
    {
      v_prey = Coords{0., 0.};
    }
    return v_prey;
  }

  Coords create_velocity_with_rules(const Parameters &par,
                                    const Flock &stormo,
                                    const SingleBoid &b,
                                    const SingleBoid &hunter)
  {
    auto neighbours = get_neighbours(par.d, b, stormo);
    return b.getVelocity() + separation(par, neighbours, b) +
           alignment(par, neighbours, b) +
           cohesion(par, neighbours, b) +
           hunter_repulsion(hunter, b, par);
  }

} // namespace boidz
