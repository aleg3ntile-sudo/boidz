#include "rules.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace boidz
{

  float get_distance(const Boid &b1, const Boid &b2)
  {
    Coords a = b1.getPosition();
    Coords b = b2.getPosition();
    Coords c = a - b;
    assert(std::isfinite(a.x) && std::isfinite(a.y));
    assert(std::isfinite(b.x) && std::isfinite(b.y));

    float distance = static_cast<float>(std::sqrt(c.x * c.x + c.y * c.y));

    assert(distance >= 0.);
    return distance;
  }

  bool check_critical_distance(const float &d_s, const Boid &b1,
                               const Boid &b2)
  {
    assert(d_s >= 0.);
    Coords a = b1.getPosition();
    Coords b = b2.getPosition();
    Coords c = a - b;
    bool critical_distance = c.x * c.x + c.y * c.y <= d_s * d_s;
    return critical_distance;
  }

  std::vector<Boid> get_neighbours(const float &distance,
                                   const Boid &s, const Flock &f)
  {
    assert(distance >= 0.);
    std::vector<Boid> neighbours{};

    for (const auto &x : f.flock)
    {
      if (s.getCardinality() != x.getCardinality() &&
          check_critical_distance(distance, s, x))
      {
        neighbours.push_back(x);
      }
    }
    assert(neighbours.size() <= f.flock.size());
    return neighbours;
  }

  Coords separation(const Parameters &par, const std::vector<Boid> &neighbours,
                    const Boid &a)
  {
    Coords sum{0, 0};
    assert(par.d_s >= 0.);
    assert(par.s >= 0.);

    for (const auto &b : neighbours)
    {
      if (check_critical_distance(par.d_s, a, b))
      {
        sum += (b.getPosition() - a.getPosition());
      }
    }

    Coords v_separation = sum * (-par.s);
    assert(std::isfinite(v_separation.x) && std::isfinite(v_separation.y));
    return v_separation;
  }

  Coords alignment(const Parameters &par, const std::vector<Boid> &neighbours,
                   const Boid &s)
  {
    Coords v_alignment{0., 0.};
    Coords sum{0., 0.};
    auto n = static_cast<float>(neighbours.size());

    for (const auto &b : neighbours)
    {
      sum += b.getVelocity();
    }
    if (n >= 1)
    {
      v_alignment = (sum / n - s.getVelocity()) * (par.a);
      assert(par.a >= 0.);
      assert(std::isfinite(v_alignment.x) && std::isfinite(v_alignment.y));
      return v_alignment;
    }
    else
    {
      return v_alignment;
    }
  }

  Coords cohesion(const Parameters &par, const std::vector<Boid> &neighbours,
                  const Boid &s)
  {
    Coords com{0., 0.};
    Coords v_cohesion{0., 0.};
    assert(par.c >= 0.);
    auto n = static_cast<float>(neighbours.size());
    for (const auto &b : neighbours)
    {
      com += b.getPosition() / n;
    }

    if (n >= 1)
    {
      v_cohesion = (com - s.getPosition()) * (par.c);
      assert(std::isfinite(v_cohesion.x) && std::isfinite(v_cohesion.y));
      return v_cohesion;
    }
    else
    {
      return v_cohesion;
    }
  }

  Coords hunt_the_flock(const Boid &hunter, const Flock &stormo,
                        const Parameters &par)
  {
    Coords com{0., 0.};
    Coords v_hunt{0., 0.};
    assert(par.h >= 0.);
    auto s = static_cast<float>(stormo.flock.size());
    for (const auto &b : stormo.flock)
    {
      com += (b.getPosition() / s);
    }
    if (s > 0)
    {
      v_hunt = (com - hunter.getPosition()) * (par.h);
      assert(std::isfinite(v_hunt.x) && std::isfinite(v_hunt.y));
      return v_hunt;
    }
    else
    {
      return v_hunt;
    }
  }

  Coords hunt_neighbours(const Boid &hunter, const Flock &stormo,
                         const Parameters &par)
  {
    Coords v_chase_neighbours{0., 0.};
    Coords sum{0., 0.};
    auto s = static_cast<float>(stormo.flock.size());
    for (const auto &b : stormo.flock)
    {
      if (check_critical_distance(50., hunter, b))
      {
        sum += (b.getPosition() - hunter.getPosition());
        v_chase_neighbours = (sum * par.h) / s;
      }
    }
    return v_chase_neighbours;
  }

  Coords hunter_repulsion(const Boid &hunter, const Boid &prey,
                          const Parameters &par)
  {
    Coords v_prey{0., 0.};
    assert(par.p >= 0);
    if (check_critical_distance(par.d, prey, hunter))
    {
      v_prey = (hunter.getPosition() - prey.getPosition()) * (-par.p);
      assert(std::isfinite(v_prey.x) && std::isfinite(v_prey.y));
      return v_prey;
    }
    else
    {
      return v_prey;
    }
  }

  Coords create_velocity_with_rules(const Parameters &par,
                                    const Flock &stormo,
                                    const Boid &b,
                                    const Boid &hunter)
  {
    auto neighbours = get_neighbours(par.d, b, stormo);
    return b.getVelocity() + separation(par, neighbours, b) +
           alignment(par, neighbours, b) +
           cohesion(par, neighbours, b) +
           hunter_repulsion(hunter, b, par);
  }

  void update_flock(Flock &stormo, const Parameters &params, const Boid &hunter, float time)
  {
    std::vector<Coords> updated_velocities(stormo.flock.size());
    for (size_t i = 0; i < stormo.flock.size(); ++i)
    {
      updated_velocities[i] = create_velocity_with_rules(params, stormo,
                                                         stormo.flock[i], hunter);
    }

    for (size_t i = 0; i < stormo.flock.size(); ++i)
    {
      stormo.flock[i].update_velocity_with_rules(updated_velocities[i]);
      stormo.flock[i].update_position_in_time(time);
      stormo.flock[i].border_restriction(800., 600.);
      stormo.flock[i].speed_restriction(130);
      stormo.flock[i].update_shape();
    }
  }

  void update_hunter(Boid &hunter, const Flock &flock, const Parameters &params, float time)
  {
    {
      Coords new_hunt_velocity = hunt_the_flock(hunter, flock, params) +
                                 hunt_neighbours(hunter, flock, params);
      hunter.update_velocity_with_rules(new_hunt_velocity);
      hunter.update_position_in_time(time);
      hunter.border_restriction(800.f, 600.f);
      hunter.speed_restriction(100.f);
      hunter.update_shape();
    }
  }

} // namespace boidz
