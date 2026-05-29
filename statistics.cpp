#include "stats.hpp"
#include <cmath>

using namespace boidz;

void speed_stats(Flock const &stormo, double const &time,
                 sf::RenderWindow &window2) {

  float margine_x = 50.f;
  float margine_y = 50.f;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);

  sf::Vector2f origin(margine_x, 600.f - margine_y);

  sf::VertexArray axis(sf::PrimitiveType::Lines, 4);
  sf::Color axis_color = sf::Color::White;

  axis[0].position = origin;
  axis[0].color = axis_color;
  axis[1].position = sf::Vector2f(800.f - 50.f, origin.y);
  axis[1].color = axis_color;

  axis[2].position = origin;
  axis[2].color = axis_color;
  axis[3].position = sf::Vector2f(origin.x, 50.f);
  axis[3].color = axis_color;

  sf::Clock clock;

  std::vector<Coords> vector_of_mean_speeds{};

  while (window2.isOpen()) {

    sf::Event event;
    while (window2.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window2.close();
      }
      Coords mean_speed_for_cycle = mean_velocity(stormo);
      vector_of_mean_speeds.push_back(mean_speed_for_cycle);

      for (auto &s : vector_of_mean_speeds) {
        double output_speed = std::sqrt(std::pow(s.x, 2) + std::pow(s.y, 2));
        point.setPosition(time + 50.f, output_speed);
      }
      window2.clear();

      window2.draw(axis);
      window2.draw(point);
      window2.display();
    }
  }
}