#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "rules.hpp"
using namespace boidz;
/* struct con elementi utilizzabili in più test per non doverli ricreare ogni
volta
struct MocksFlock {
  Flock stormo_vuoto;
  Flock solitario{1};
  Flock mock_stormo{10};
};

struct MocksBoid {
  SingleBoid uccello_fermo;
};

struct MocksPar {}; */

TEST_CASE("check get_distance function") {
  SingleBoid b1{{1.0, 2.0}, {0.5, 0.5}, 3};
  SingleBoid b2{{3.0, 1.5}, {1.0, 0.2}, 12};
  CHECK(get_distance(b1, b2) == doctest::Approx(2.06).epsilon(0.01));
}

TEST_CASE("check get_distance function") {
  SingleBoid b1{{1.0, 2.3}, {3.1, 2.0}, 1};
  SingleBoid b2;
  CHECK(get_distance(b1, b2)== doctest::Approx(2.51).epsilon(0.01));
}

TEST_CASE("check check_critical_distance function") {
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{2.0, 0.0}, {0.0, 0.0}, 1};
  CHECK(check_critical_distance(2.0, b1, b2) == true);
  CHECK(check_critical_distance(1.0, b1, b2) == false);
}

TEST_CASE("check check_neighbours function") {
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{3.0, 4.0}, {0.0, 0.0}, 1};
  CHECK(check_neighbours(5.0, b1, b2) == true);
  CHECK(check_neighbours(4.9, b1, b2) == false);
}

TEST_CASE("check get_neighbours function - no neighbours") {
  Flock f;
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{10.0, 0.0}, {0.0, 0.0}, 2};
  SingleBoid b3{{20.0, 0.0}, {0.0, 0.0}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0, b1, f);
  CHECK(result.size() == 0);
}

TEST_CASE("check get_neighbours function - some neighbours") {
  Flock f;
  SingleBoid b1{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b2{{1.0, 0.0}, {0.0, 0.0}, 2};
  SingleBoid b3{{10.0, 0.0}, {0.0, 0.0}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0, b1, f);
  CHECK(result.size() == 1);
}

TEST_CASE("check separation-for_singleboid function") {
  Parameters par{1.0, 5.0, 2.0, 0.0, 0.0}; // s=1, d=5, d_s=2
  Flock f;
  SingleBoid a{{0.0, 0.0}, {0.0, 0.0}, 1};
  SingleBoid b{{1.0, 0.0}, {0.0, 0.0}, 2};
  f.flock.push_back(a);
  f.flock.push_back(b);

  Coords result = separation_for_single_boid(par, f, a);

  CHECK(result.x == doctest::Approx(-1.0));
  CHECK(result.y == doctest::Approx(0.0));
}