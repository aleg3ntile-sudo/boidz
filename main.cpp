#include <SFML/Graphics.hpp>
#include <algorithm>
#include <random>

#include "boidz.hpp"
#include "rules.hpp"
#include "stats.hpp"

using namespace boidz;
int main() {
  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids");
  window.setFramerateLimit(60);

  sf::RenderWindow window2(sf::VideoMode(500, 600), "Statistics");
  window2.setFramerateLimit(60);

  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> px{0., 100.};
  std::uniform_real_distribution<double> py{0., 100.};
  std::uniform_real_distribution<double> vel{-50., 50.};

  for (int j{0}; j != 20; ++j) {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  SingleBoid hunter{{400., 300.}, {0., 0.}, 104};
  hunter.setColor(sf::Color(255, 100, 100));

  Parameters params{1.2, 50.0, 7.5, 0.3, 0.02, 1., 0.5};

  sf::Clock clock;

  float margine_x = 50.f;
  float margine_y = 50.f;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);

  sf::Vector2f origin(margine_x, 600.f - margine_y);

  sf::VertexArray axis(sf::PrimitiveType::Lines, 4);
  sf::Color axis_color = sf::Color::White;

  axis[0].position = origin;
  axis[0].color = axis_color;
  axis[1].position = sf::Vector2f(500.f - 50.f, origin.y);
  axis[1].color = axis_color;

  axis[2].position = origin;
  axis[2].color = axis_color;
  axis[3].position = sf::Vector2f(origin.x, 50.f);
  axis[3].color = axis_color;

  const size_t max_points =
      450; // numero massimo di punti sul grafico per veitare oveflow
  std::vector<Coords> vector_of_mean_speeds{};

  double timer = 0.;
  const double delay = 5;
  while (window.isOpen() || window2.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    while (window2.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window2.close();
      }
    }

    if (window.isOpen()) {
      double dt = clock.restart().asSeconds();
      timer += dt;
      std::vector<Coords> updated_velocities{};
      for (auto &x : stormo.flock) {
        Coords new_velocity = create_velocity_with_rules_for_single_boid(
            params, stormo, x, hunter);
        updated_velocities.push_back(new_velocity);
      }
      for (long unsigned int i = 0; i != stormo.flock.size(); ++i) {
        stormo.flock[i].update_Velocity_with_rules_for_single_boid(
            updated_velocities[i]);
        stormo.flock[i].update_Position_in_time_for_single_boid(dt);
        stormo.flock[i].BorderRestriction(800., 600.);
        stormo.flock[i].SpeedRestriction(130);
        stormo.flock[i].update_Shape();
      }

      if (timer >= delay) {
        Coords new_hunt_velocity = hunt(hunter, stormo, params);
        hunter.update_Velocity_with_rules_for_single_boid(new_hunt_velocity);
        hunter.update_Position_in_time_for_single_boid(dt);
        hunter.BorderRestriction(800., 600.);
        hunter.SpeedRestriction(100);
        hunter.update_Shape();
      }

      Coords mean_speed_for_cycle = mean_velocity(stormo);
      vector_of_mean_speeds.push_back(mean_speed_for_cycle);
      if (vector_of_mean_speeds.size() > max_points) {
        vector_of_mean_speeds.erase(
            vector_of_mean_speeds.begin(),
            vector_of_mean_speeds.begin() +
                (vector_of_mean_speeds.size() -
                 max_points)); // toglie i punti extra dal grafico
      }

      window.clear(sf::Color(15, 17, 26));
      for (auto &b : stormo.flock) {
        b.draw(window);
      }
      hunter.draw(window);
      window.display();
    }

    if (window2.isOpen()) {
      window2.clear();
      window2.draw(axis);

      int visible_count = vector_of_mean_speeds.size();
      double graph_left = 50.0;
      double graph_right = 450.0;
      double graph_width = graph_right - graph_left;
      double x_step;
      if (visible_count > 1) {
        x_step = graph_width / (visible_count - 1);
      } else {
        x_step = 0.0;
      }

      for (int i = 0; i < visible_count; ++i) {
        double output_speed =
            std::sqrt(std::pow(vector_of_mean_speeds[i].x, 2) +
                      std::pow(vector_of_mean_speeds[i].y, 2));
        double x = graph_left + x_step * i;
        double y = 550.0 - output_speed;
        if (y < 50.0) {
          y = 50.0;
        }
        if (y > 550.0) {
          y = 550.0;
        }
        point.setPosition((float)x, (float)y);
        window2.draw(point);
      }
      window2.display();
    }
  }
}
