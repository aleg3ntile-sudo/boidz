#include "boidz.hpp"

namespace boidz {

Coords operator+(const Coords &p, const Coords &q) {
  return {p.x + q.x, p.y + q.y};
}

sf::ConvexShape createBoidShape(float size) {
  sf::ConvexShape shape;
  shape.setPointCount(3);

  shape.setPoint(0, sf::Vector2f(size, 0.));
  shape.setPoint(1, sf::Vector2f(-size * 0.6, -size * 0.5));
  shape.setPoint(2, sf::Vector2f(-size * 0.6, size * 0.5));
  shape.setFillColor(sf::Color(100, 200, 255));

  shape.setOrigin(0., 0.);

  return shape;
}

SingleBoid::SingleBoid()
    : position{}, velocity{}, cardinality(0), shape{createBoidShape(5.)} {}

SingleBoid::SingleBoid(Coords p, Coords v, int i)
    : position(p), velocity(v), cardinality(i), shape{createBoidShape(5.)} {}

int SingleBoid::getCardinality() const { return cardinality; }

Coords SingleBoid::getPosition() const { return position; }

Coords SingleBoid::getVelocity() const { return velocity; }

void SingleBoid::update_Position_in_time_for_single_boid(float time) {
  position.x = position.x + time * velocity.x;
  position.y = position.y + time * velocity.y;
}

void SingleBoid::update_Shape() {
  float angle = std::atan2(velocity.y, velocity.x) * 180 / M_PI;
  shape.setPosition(sf::Vector2f(position.x, position.y));
  shape.setRotation(angle);
}

void SingleBoid::update_Velocity_with_rules_for_single_boid(
    Coords updated_velocity) {
  velocity = updated_velocity;
}

void SingleBoid::BorderRestriction(float width, float height) {
  if (position.x < 0.)
    position.x += width;
  else {
    if (position.x >= width) {
      position.x -= width;
    }
  }

  if (position.y < 0.)
    position.y += height;
  else {
    if (position.y >= height) {
      position.y -= height;
    }
  }
}

void SingleBoid::SpeedRestriction(float speed_limit) {
  float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
  if (speed > speed_limit) {
    velocity.x = velocity.x * speed_limit / speed;
    velocity.y = velocity.y * speed_limit / speed;
  }
}

void SingleBoid::draw(sf::RenderWindow &window) const { window.draw(shape); }

void SingleBoid::setColor(const sf::Color &c) { shape.setFillColor(c); }

} // namespace boidz
