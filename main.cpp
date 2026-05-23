#include "boidz.hpp"
#include "rules.hpp"
#include "stats.cpp"
#include <random>

using namespace boidz;

int main()
{
  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> uniform{0., 1.};

  for (int j{0}; j != 10; ++j)
  {
    Coords p{uniform(eng), uniform(eng)};
    Coords v{uniform(eng), uniform(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  check_status(stormo);

  std::cout << "Mean velocity: " << mean_velocity(stormo).x << " " << mean_velocity(stormo).y << '\n'
            << "Centre of mass: " << centre_of_mass_position(stormo).x << " " << centre_of_mass_position(stormo).y << '\n';
}