#include <SFML/Graphics.hpp>
#include <random>

#include "boidz.hpp"
#include "rules.hpp"
#include "stats.cpp"
#include <cmath>

// ipotetico main smfl
using namespace boidz;
int main() {
  sf::RenderWindow window(
      sf::VideoMode(800, 600),  // inizializzerò poi height e width (o lasciamo
                                // sempre fissi 600-800?)
      "Boids");                 // informazioni sulla finestra grafica
  window.setFramerateLimit(60); // fps

  sf::RenderWindow window2(sf::VideoMode(800, 600), "Statistics");
  window.setFramerateLimit(60);

  sf::CircleShape dot(1.f);
  dot.setFillColor(sf::Color::Red);

  // main originale
  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> px{0., 800.};
  // importante avere come limiti le dimensioni dello schermo
  std::uniform_real_distribution<double> py{0., 600.};
  std::uniform_real_distribution<double> vel{-50., 50.};
  // che velocità sarebbe indicata?
  for (int j{0}; j != 100; ++j) {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  SingleBoid hunter{{400., 300.}, {0., 0.}, 104};

  // for (auto &x : stormo.flock)
  //{
  //     // per settare posizioni, scalare (è un metodo)
  //     x.setupSprite(texture);
  // }

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
  axis[1].position = sf::Vector2f(800.f - 50.f, origin.y);
  axis[1].color = axis_color;

  axis[2].position = origin;
  axis[2].color = axis_color;
  axis[3].position = sf::Vector2f(origin.x, 50.f);
  axis[3].color = axis_color;

  std::vector<Coords> vector_of_mean_speeds{};

  while (window.isOpen() || window2.isOpen()) {

    sf::Event event;
    while (window.pollEvent(event)) {

      if (event.type == sf::Event::Closed) {
        window.close();
      } else {
        
        double dt = clock.restart().asSeconds();
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
          stormo.flock[i].aggiornaSprite();
        }

        Coords new_hunt_velocity = hunt(hunter, stormo, params);
        hunter.update_Velocity_with_rules_for_single_boid(new_hunt_velocity);
        hunter.update_Position_in_time_for_single_boid(dt);
        hunter.BorderRestriction(800., 600.);
        hunter.SpeedRestriction(100);
        hunter.aggiornaSprite();

        // render
        window.clear(sf::Color(15, 17, 26)); // blu notte
        for (auto &b : stormo.flock) {
          dot.setPosition(b.getPosition().x, b.getPosition().y);
          // b.draw(window);
          window.draw(dot);
        }
        dot.setPosition(hunter.getPosition().x, hunter.getPosition().y);
        window.draw(dot);

        window.display();
      }
    }

    sf::Event event2;
    while (window2.pollEvent(event2)) {
      if (event2.type == sf::Event::Closed) {
        window2.close();
      } else {
        double dt2 = clock.restart().asSeconds();
        Coords mean_speed_for_cycle = mean_velocity(stormo);
        vector_of_mean_speeds.push_back(mean_speed_for_cycle);

        window2.clear();

        window2.draw(axis);
        for (auto &s : vector_of_mean_speeds) {
          double output_speed = std::sqrt(std::pow(s.x, 2) + std::pow(s.y, 2));
          point.setPosition(dt2 + 50.f, output_speed);
          window2.draw(point);
        }
        window2.display();
      }
    }
  }
}
