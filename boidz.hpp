#ifndef BOIDZ_HPP
#define BOIDZ_HPP

#include <SFML/Graphics.hpp>
#include "parameters.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <random>

namespace boidz {
struct Coords {
  float x;
  float y;

  Coords() = default;
  Coords(float x_, float y_) : x(x_), y(y_) {}
};

Coords operator+(const Coords &p, const Coords &q);
Coords operator-(const Coords &p, const Coords &q);
Coords operator*(const Coords &p, const float &a);
Coords operator/(const Coords &p, const float &a);
Coords operator+=(Coords &p, const Coords &q);

sf::ConvexShape create_boid_shape(float size);

class Boid {
  Coords position;
  Coords velocity;
  int cardinality;
  sf::ConvexShape shape;

public:
  Boid();
  Boid(Coords p, Coords v, int i);
  int getCardinality() const;
  Coords getPosition() const;
  Coords getVelocity() const;

  void update_position_in_time(float time);
  void update_velocity_with_rules(Coords updated_velocity);
  void border_restriction(float width, float height);
  void speed_restriction(float speed_limit);
  void update_shape();
  void draw(sf::RenderWindow &window) const;
  void set_color(const sf::Color &c);
};

class Flock {
public:
  std::vector<Boid> flock{};
  Flock() : flock{} {}
  Flock(long unsigned int N) : flock(N) {}

  void generate_boids(const int &size);
};

} // namespace boidz

#endif
