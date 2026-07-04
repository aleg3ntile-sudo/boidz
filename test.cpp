#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <random>

#include "doctest.h"
#include "rules.hpp"
#include "stats.hpp"

using namespace boidz;

// operator
TEST_CASE("check + operator on Coords")
{
  Coords a{1.f, -9.f};
  Coords b{8.f, 2.3f};
  Coords result = a + b;
  CHECK(result.x == doctest::Approx(9.00f));
  CHECK(result.y == doctest::Approx(-6.7f));
}

TEST_CASE("check - operator on Coords")
{
  Coords a{2.f, 3.f};
  Coords b{-3.f, 2.f};
  Coords result = a - b;
  CHECK(result.x == doctest::Approx(5.f));
  CHECK(result.y == doctest::Approx(1.f));
}

TEST_CASE("check * operator on Coords")
{
  float a{3.4f};
  Coords b{8.f, -2.3f};
  Coords result = b * a;
  CHECK(result.x == doctest::Approx(27.2f));
  CHECK(result.y == doctest::Approx(-7.82f));
}

TEST_CASE("check * operator on Coords")
{
  float a{0.f};
  Coords b{8.f, 2.3f};
  Coords result = b * a;
  CHECK(result.x == doctest::Approx(0.f));
  CHECK(result.y == doctest::Approx(0.f));
}

TEST_CASE("check / operator on Coords")
{
  float a{3.4f};
  Coords b{10.f, -2.3f};
  Coords result = b / a;
  CHECK(result.x == doctest::Approx(2.94117f));
  CHECK(result.y == doctest::Approx(-0.67647f));
}

TEST_CASE("check += operator on Coords")
{
  Coords result{2.f, 3.f};
  for (float i{0.f}; i < 5; ++i)
  {
    Coords addend{i, 2 * i};
    result += addend;
  }
  CHECK(result.x == doctest::Approx(12.f));
  CHECK(result.y == doctest::Approx(23.f));
}

// get_distance
TEST_CASE("check get_distance function")
{
  Boid b1{{1.0f, 2.0f}, {0.5f, 0.5f}, 3};
  Boid b2{{3.0f, 1.5f}, {1.0f, 0.2f}, 12};
  CHECK(get_distance(b1, b2) == doctest::Approx(2.06f).epsilon(0.01f));
}

TEST_CASE("check get_distance function")
{
  Boid b1{{1.0f, 2.3f}, {3.1f, 2.0f}, 1};
  Boid b2;
  CHECK(get_distance(b1, b2) == doctest::Approx(2.51f).epsilon(0.01f));
}

TEST_CASE("check get_distance function - one boid")
{
  Boid b1{{0.0f, 1.0f}, {4.3f, 2.0f}, 1};
  CHECK(get_distance(b1, b1) == 0.f);
}

// check_critical_distance
TEST_CASE("check check_critical_distance function")
{
  Boid b1{{0.0f, 0.0f}, {0.0f, 0.0f}, 1};
  Boid b2{{2.0f, 0.0f}, {0.0f, 0.0f}, 1};
  CHECK(check_critical_distance(2.0f, b1, b2) == true);
  CHECK(check_critical_distance(1.0f, b1, b2) == false);
}

// get_neighbours
TEST_CASE("check get_neighbours function - no neighbours")
{
  Flock f;
  Boid b1{{0.0f, 0.0f}, {0.0f, 0.0f}, 1};
  Boid b2{{10.0f, 0.0f}, {0.0f, 0.0f}, 2};
  Boid b3{{20.0f, 0.0f}, {0.0f, 0.0f}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0f, b1, f);
  CHECK(result.size() == 0);
}

TEST_CASE("check get_neighbours function - some neighbours")
{
  Flock f;
  Boid b1{{0.0f, 0.0f}, {0.0f, 0.0f}, 1};
  Boid b2{{1.0f, 0.0f}, {0.0f, 0.0f}, 2};
  Boid b3{{10.0f, 0.0f}, {0.0f, 0.0f}, 3};
  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0f, b1, f);
  CHECK(result.size() == 1);
}

TEST_CASE("check get_neighbours function - all are neighbours")
{
  Flock f;
  Boid b1{{0.0f, 0.0f}, {0.5f, 0.0f}, 1};
  Boid b2{{0.0f, 1.0f}, {0.0f, 0.5f}, 2};
  Boid b3{{1.0f, 0.0f}, {0.0f, 1.0f}, 3};

  f.flock.push_back(b1);
  f.flock.push_back(b2);
  f.flock.push_back(b3);

  auto result = get_neighbours(2.0f, b1, f);
  CHECK(result.size() == 2);
}

// separation
TEST_CASE("check separation_for_Boid function")
{
  Parameters par{5.0f, 5.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{0.0f, 0.0f}, {0.0f, 0.0f}, 1};
  Boid b{{1.0f, 0.0f}, {0.0f, 0.0f}, 2};
  neighbours.push_back(b);

  Coords result = separation(par, neighbours, a);

  CHECK(result.x == doctest::Approx(-1.0f));
  CHECK(result.y == doctest::Approx(0.0f));
}

TEST_CASE("check separation_for_Boid function - one boid is too distant")
{
  Parameters par{5.0f, 5.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{0.0f, 0.0f}, {0.0f, 0.0f}, 1};
  Boid b{{1.0f, 0.0f}, {0.0f, 0.0f}, 2};
  Boid c{{40.0f, 0.0f}, {0.0f, 0.0f}, 3};
  neighbours.push_back(b);
  neighbours.push_back(c);

  Coords result = separation(par, neighbours, a);

  CHECK(result.x == doctest::Approx(-1.0f));
  CHECK(result.y == doctest::Approx(0.0f));
}

// alignment
TEST_CASE("check alignment function")
{
  Parameters par{5.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.0f, 0.0f}, {1.0f, 0.0f}, 1};
  Boid b{{0.0f, 1.0f}, {2.0f, 1.0f}, 2};
  neighbours.push_back(b);
  Coords result = alignment(par, neighbours, a);

  CHECK(result.x == doctest::Approx(1.0f));
  CHECK(result.y == doctest::Approx(1.0f));
}

TEST_CASE("check alignment function - empty neighbours")
{
  Parameters par{0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.5f, 0.5f}, {1.0f, -1.0f}, 1};
  Coords result = alignment(par, neighbours, a);

  CHECK(result.x == doctest::Approx(0.0f));
  CHECK(result.y == doctest::Approx(0.0f));
}

TEST_CASE("check alignment function - single neighbour")
{
  Parameters par{0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.5f, 0.5f}, {1.0f, -1.0f}, 1};
  Boid b{{-1.0f, 0.0f}, {2.0f, 3.0f}, 2};
  neighbours.push_back(a);
  Coords result = alignment(par, neighbours, b);

  CHECK(result.x == doctest::Approx(-2.0f));
  CHECK(result.y == doctest::Approx(-8.0f));
}

// cohesion
TEST_CASE("check cohesion function")
{
  Parameters par{5.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.0f, -2.0f}, {0.0f, 1.0f}, 1};
  Boid b{{-1.0f, 2.5f}, {1.0f, 1.0f}, 2};
  neighbours.push_back(b);
  Coords result = cohesion(par, neighbours, a);

  CHECK(result.x == doctest::Approx(-2.0f));
  CHECK(result.y == doctest::Approx(4.5f));
}

TEST_CASE("check cohesion function - empty neighbours")
{
  Parameters par{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.0f, -2.0f}, {0.0f, 1.0f}, 1};
  Coords result = cohesion(par, neighbours, a);

  CHECK(result.x == doctest::Approx(0.0f));
  CHECK(result.y == doctest::Approx(0.0f));
}

TEST_CASE("check cohesion function - multiple neighbours")
{
  Parameters par{5.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  std::vector<Boid> neighbours;
  Boid a{{1.0f, -2.0f}, {0.0f, 1.0f}, 1};
  Boid b{{-1.0f, 2.5f}, {1.0f, 1.0f}, 2};
  Boid c{{67.0f, 1.0f}, {1.0f, 1.0f}, 3};
  neighbours.push_back(b);
  neighbours.push_back(c);
  Coords result = cohesion(par, neighbours, a);

  CHECK(result.x == doctest::Approx(32.0f));
  CHECK(result.y == doctest::Approx(3.75f));
}

// hunt
TEST_CASE("check hunt function - no boids")
{
  Boid h{{0.f, 0.f}, {0.f, 0.f}, 1};
  Parameters par{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};
  Flock f;
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == (0.f));
  CHECK(result.y == (0.f));
}

TEST_CASE("check hunt function - three boids")
{
  Parameters par{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};
  Boid h{{7.0f, 1.0f}, {8.0f, 5.0f}, 11};
  Flock f;
  Boid a{{6.f, 7.f}, {0.f, 1.f}, 1};
  Boid b{{1.f, 1.f}, {1.5f, 0.5f}, 2};
  Boid c{{0.f, 1.f}, {1.3f, 0.8f}, 3};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == doctest::Approx(-4.666666f));
  CHECK(result.y == doctest::Approx(2.0f));
}

TEST_CASE("check hunt function - simmetrical configuration")
{
  Parameters par{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 13.0f, 1.0f};
  Boid h{{0.0f, 0.0f}, {8.0f, 5.0f}, 11};
  Flock f;
  Boid a{{-4.f, 0.f}, {0.f, 1.f}, 1};
  Boid b{{4.f, 0.f}, {2.5f, 1.2f}, 2};
  Boid c{{0.f, -4.f}, {1.3f, 0.8f}, 3};
  Boid d{{0.f, 4.f}, {-3.9f, 1.1f}, 4};
  f.flock.push_back(a);
  f.flock.push_back(b);
  f.flock.push_back(c);
  f.flock.push_back(d);
  Coords result = hunt_the_flock(h, f, par);
  CHECK(result.x == 0.f);
  CHECK(result.y == 0.f);
}

// hunter_repulsion
TEST_CASE("check hunter_repulsion function - no hunter in sight")
{
  Parameters par{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 13.0f, 3.0f};
  Boid h{{30.f, 40.f}, {0.0f, 5.0f}, 11};
  Flock f;
  Boid a{{2., 1.}, {0., 1.}, 1};
  f.flock.push_back(a);
  Coords result = hunter_repulsion(h, a, par);
  CHECK(result.x == 0.f);
  CHECK(result.y == 0.f);
}

TEST_CASE("check hunter_repulsion function - hunter in sight")
{
  Parameters par{15.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  Boid h{{10.0f, 10.0f}, {0.5f, 1.0f}, 1};
  Boid prey{{5.5f, 1.0f}, {-4.0f, 2.0f}, 11};
  Coords result = hunter_repulsion(h, prey, par);
  CHECK(result.x == -4.5f);
  CHECK(result.y == -9.0f);
}

// create_velocity_with_rules
TEST_CASE("check create_velocity_with_rules function")
{
  Parameters par{5.0f, 1.0f, 10.0f, 1.0f, 3.0f, 0.0f, 0.0f};
  Boid a{{-3.0f, -2.0f}, {5.0f, 2.0f}, 1};
  Boid b{{1.0f, 1.0f}, {10.0f, 1.0f}, 2};
  Flock f;
  f.flock.push_back(a);
  f.flock.push_back(b);
  Boid hunter{{500.f, 500.0f}, {0.0f, 0.0f}, 11};
  Coords result = create_velocity_with_rules(par, f, b, hunter);
  CHECK(result.x == doctest::Approx(-7.f));
  CHECK(result.y == doctest::Approx(-7.f));
}

// vector_of_speeds
TEST_CASE("get_vector_of_speeds: empty flock returns empty vector")
{
  Flock f{};
  auto result = get_vector_of_speeds(f);
  CHECK(result.empty());
}

TEST_CASE("get_vector_of_speeds: two boids returns correct speeds")
{
  Flock f{};
  f.flock.push_back(Boid{{0.f, 0.f}, {3.f, 4.f}, 0});
  f.flock.push_back(Boid{{0.f, 0.f}, {1.f, 2.f}, 1});

  auto result = get_vector_of_speeds(f);
  CHECK(result.size() == 2);
  CHECK(result[0] == doctest::Approx(5.f));
  CHECK(result[1] == doctest::Approx(std::sqrt(5.f)));
}

// vector_of_distances
TEST_CASE("get_vector_of_distances: empty flock returns empty vector")
{
  Flock f{};
  auto result = get_vector_of_distances(f);
  CHECK(result.empty());
}

TEST_CASE("get_vector_of_distances: two boids return one distance")
{
  Flock f{};
  f.flock.push_back(Boid{{0.f, 0.f}, {0.f, 0.f}, 0});
  f.flock.push_back(Boid{{3.f, 4.f}, {0.f, 0.f}, 1});

  auto result = get_vector_of_distances(f);
  CHECK(result.size() == 1);
  CHECK(result[0] == doctest::Approx(5.f));
}

TEST_CASE("get_vector_of_distances: distances calculated")
{
  Flock f{};
  f.generate_boids(10);

  auto result = get_vector_of_distances(f);
  CHECK(result.size() == 45);
}

// mean and error
TEST_CASE("testing the mean ad error function: wrong word in function call")
{
  Flock f{};
  f.generate_boids(5);

  Coords result = mean_and_error(f, "hello");
  CHECK(result.x == -1.f);
  CHECK(result.y == -1.f);
}

TEST_CASE("testing the mean ad error function: trivial case")
{
  Flock example{};
  Boid b1{{10.f, 10.f}, {3.5f, 3.5f}, 1};
  Boid b2{{10.f, 10.f}, {3.5f, 3.5f}, 2};
  Boid b3{{10.f, 10.f}, {3.5f, 3.5f}, 3};
  Boid b4{{10.f, 10.f}, {3.5f, 3.5f}, 4};
  Boid b5{{10.f, 10.f}, {3.5f, 3.5f}, 5};
  Boid b6{{10.f, 10.f}, {3.5f, 3.5f}, 6};
  Boid b7{{10.f, 10.f}, {3.5f, 3.5f}, 7};
  example.flock.push_back(b1);
  example.flock.push_back(b2);
  example.flock.push_back(b3);
  example.flock.push_back(b4);
  example.flock.push_back(b5);
  example.flock.push_back(b6);
  example.flock.push_back(b7);

  Coords result = mean_and_error(example, "speed");
  Coords result2 = mean_and_error(example, "distance");

  CHECK(result.x == doctest::Approx(4.94975f));
  CHECK(result.y == doctest::Approx(1.6712f));
  CHECK(result2.x == doctest::Approx(0.f));
  CHECK(result2.y == doctest::Approx(0.f));
}

TEST_CASE("testing the mean ad error function: trivial case")
{
  Flock f{};
  auto result = mean_and_error(f, "distance");

  CHECK(result.x == 0.f);
  CHECK(result.y == 0.f);
}
