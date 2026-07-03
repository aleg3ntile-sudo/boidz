#include "boidz.hpp"
#include <cassert>

namespace boidz
{

  Coords operator+(const Coords &p, const Coords &q)
  {
    return {p.x + q.x, p.y + q.y};
  }

  Coords operator*(const Coords &p, const float &a)
  {
    return {p.x * a, p.y * a};
  }

  Coords operator-(const Coords &p, const Coords &q)
  {
    return {p.x - q.x, p.y - q.y};
  }

  Coords operator/(const Coords &p, const float &a)
  {
    assert(a != 0.f);
    return {p.x / a, p.y / a};
  }

  Coords operator+=(Coords &p, const Coords &q)
  {
    return p = p + q;
  }

  sf::ConvexShape create_boid_shape(float size)
  {
    sf::ConvexShape shape;
    shape.setPointCount(3);

    shape.setPoint(0, sf::Vector2f(size, 0.));
    shape.setPoint(1, sf::Vector2f(-size * 0.6f, -size * 0.5f));
    shape.setPoint(2, sf::Vector2f(-size * 0.6f, size * 0.5f));
    shape.setFillColor(sf::Color(100, 200, 255));

    shape.setOrigin(0., 0.);

    return shape;
  }

  Boid::Boid()
      : position{}, velocity{}, cardinality(0), shape{create_boid_shape(5.)} {}

  Boid::Boid(Coords p, Coords v, int i)
      : position(p), velocity(v), cardinality(i), shape{create_boid_shape(5.)} {}

  int Boid::getCardinality() const
  {
    return cardinality;
  }

  Coords Boid::getPosition() const
  {
    return position;
  }

  Coords Boid::getVelocity() const
  {
    return velocity;
  }

  void Boid::update_position_in_time(float time)
  {
    position = position + (velocity * time);
  }

  void Boid::update_shape()
  {
    float angle = static_cast<float>(std::atan2(velocity.y, velocity.x) * 180 / M_PI);
    shape.setPosition(sf::Vector2f(position.x, position.y));
    shape.setRotation(angle);
  }

  void Boid::update_velocity_with_rules(
      Coords updated_velocity)
  {
    velocity = updated_velocity;
  }

  void Boid::border_restriction(float width, float height)
  {
    if (position.x < 0.)
      position.x += width;
    else
    {
      if (position.x >= width)
      {
        position.x -= width;
      }
    }

    if (position.y < 0.)
      position.y += height;
    else
    {
      if (position.y >= height)
      {
        position.y -= height;
      }
    }
  }

  void Boid::speed_restriction(float speed_limit)
  {
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > speed_limit)
    {
      velocity = velocity * (speed_limit / speed);
    }
  }

  void Boid::draw(sf::RenderWindow &window) const
  {
    window.draw(shape);
  }

  void Boid::set_color(const sf::Color &c)
  {
    shape.setFillColor(c);
  }

  void Flock::generate_boids(const int &size){
  
  std::default_random_engine eng;
  std::uniform_real_distribution<float> pos{0., 200.};
  std::uniform_real_distribution<float> vel{-50., 50.};

  for (int j{0}; j != size; ++j) {
    Coords p{pos(eng), pos(eng)};
    Coords v{vel(eng), vel(eng)};
    Boid b = Boid(p, v, j);
    flock.push_back(b);
  }
  }

} // namespace boidz
