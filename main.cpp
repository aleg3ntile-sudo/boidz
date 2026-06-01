#include "boidz.hpp"
#include "rules.hpp"
#include "stats.hpp"
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <random>
#include <iostream>
#include <string>
#include <stdexcept>

template <typename T>
T read_parameters(const std::string &name, T min, T max)
{
  T p;

  std::cout << "Insert " << name << "between " << min << " and " << max << '\n';
  std::cin >> p;
  if (p < min || p > max)
  {
    throw std::runtime_error(name + "fuori intervallo");
  }
  return p;
}

using namespace boidz;
int main()
{
  int flock_size{};
  Parameters params{};
  try
  {
    params.d = read_parameters("Neighbours distance ", 20., 100.);
    params.d_s = read_parameters("Critical distance ", 5., 25.);
    params.s = read_parameters("Separation parameter ", 0.5, 5.);
    params.a = read_parameters("Alignment parameter ", 0.05, 1.);
    params.c = read_parameters("Cohesion parameter ", 0.01, 0.05);
    flock_size = read_parameters("Flock size ", 1, 100);
    params.h = 1.;
    params.p = 0.5;
  }
  catch (const std::runtime_error &e)
  {
    std::cerr << "Errore" << '\n';
    return EXIT_FAILURE;
  }

  Flock stormo{};
  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<float> px{0., 200.};
  std::uniform_real_distribution<float> py{0., 200.};
  std::uniform_real_distribution<float> vel{-50., 50.};

  for (int j{0}; j != flock_size; ++j)
  {
    Coords p{px(eng), py(eng)};
    Coords v{vel(eng), vel(eng)};
    SingleBoid b = SingleBoid(p, v, j);
    stormo.flock.push_back(b);
  }

  SingleBoid hunter{{400., 300.}, {0., 0.}, -1};
  hunter.setColor(sf::Color(255, 100, 100));

  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids");
  window.setFramerateLimit(60);

  sf::RenderWindow window2(sf::VideoMode(500, 600), "Statistics");
  window2.setFramerateLimit(60);


  sf::Clock clock;

  float margine_x = 50.f;
  float margine_y = 50.f;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);
  sf::CircleShape dot(2.f);
  dot.setFillColor(sf::Color::Green);

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

  sf::RectangleShape box(sf::Vector2f(100.f, 60.f));
  box.setPosition(400.f, 0.f);
  box.setFillColor(sf::Color::White);
  sf::Text label;
  sf::Text label2;
  sf::Font font;
  label.setFont(font);
  label2.setFont(font);
  label.setCharacterSize(12);
  label2.setCharacterSize(12);
  label.setFillColor(sf::Color::Black);
  label2.setFillColor(sf::Color::Black);
  label.setPosition(300.f, 10.f);
  label2.setPosition(300.f, 20.f);

  const size_t max_points = 450;
  std::vector<Coords> vector_of_mean_speeds{};
  std::vector<float> vector_of_distances{};
  std::vector<float> vector_of_mean_distances{};

  float timer = 0.;
  const float delay = 5.;
  while (window.isOpen() || window2.isOpen())
  {
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window.close();
      }
    }
    while (window2.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window2.close();
      }
    }

    if (window.isOpen())
    {
      float dt = clock.restart().asSeconds();
      timer += dt;
      std::vector<Coords> updated_velocities{};
      for (auto &x : stormo.flock)
      {
        Coords new_velocity = create_velocity_with_rules_for_single_boid(
            params, stormo, x, hunter);
        updated_velocities.push_back(new_velocity);
      }
      for (long unsigned int i = 0; i != stormo.flock.size(); ++i)
      {
        stormo.flock[i].update_Velocity_with_rules_for_single_boid(
            updated_velocities[i]);
        stormo.flock[i].update_Position_in_time_for_single_boid(dt);
        stormo.flock[i].BorderRestriction(800., 600.);
        stormo.flock[i].SpeedRestriction(130);
        stormo.flock[i].update_Shape();
      }

      if (timer >= delay)
      {
        Coords new_hunt_velocity = hunt_the_flock(hunter, stormo, params) + hunt_neighbours(hunter, stormo, params);
        hunter.update_Velocity_with_rules_for_single_boid(new_hunt_velocity);
        hunter.update_Position_in_time_for_single_boid(dt);
        hunter.BorderRestriction(800., 600.);
        hunter.SpeedRestriction(100);
        hunter.update_Shape();
      }

      Coords mean_speed_for_cycle = mean_velocity(stormo);
      vector_of_mean_speeds.push_back(mean_speed_for_cycle);
      if (vector_of_mean_speeds.size() > max_points)
      {
        vector_of_mean_speeds.erase(
            vector_of_mean_speeds.begin(),
            vector_of_mean_speeds.begin() +
                (vector_of_mean_speeds.size() -
                 max_points));
      }

      for (auto first = stormo.flock.begin(), last = stormo.flock.end() - 1; first != last; ++first)
      {
        float boidz_distance = get_distance(*first, *(first + 1));
        vector_of_distances.push_back(boidz_distance);
      }

      float mean_distance_boidz = (std::accumulate(
                                       vector_of_distances.begin(), vector_of_distances.end(), 0.) /
                                   vector_of_distances.size());
      vector_of_mean_distances.push_back(mean_distance_boidz);
      vector_of_distances.clear();
      if (vector_of_mean_distances.size() > max_points)
      {
        vector_of_mean_distances.erase(
            vector_of_mean_distances.begin(),
            vector_of_mean_distances.begin() +
                (vector_of_mean_distances.size() -
                 max_points)); // toglie i punti extra dal grafico
      }

      window.clear(sf::Color(15, 17, 26));
      for (auto &b : stormo.flock)
      {
        b.draw(window);
      }
      hunter.draw(window);
      window.display();
    }

    if (window2.isOpen())
    {
      window2.clear();
      window2.draw(axis);

      int visible_count = vector_of_mean_speeds.size();
      int visible_count2 = vector_of_mean_distances.size();
      int ii{0};
      float graph_left = 50.0;
      float graph_right = 450.0;
      float graph_width = graph_right - graph_left;
      float x_step;
      if (visible_count > 1)
      {
        x_step = graph_width / (visible_count - 1);
      }
      else
      {
        x_step = 0.0;
      }

      for (int i{0}; i < visible_count; ++i)
      {
        float output_speed =
            std::sqrt(std::pow(vector_of_mean_speeds[i].x, 2) +
                      std::pow(vector_of_mean_speeds[i].y, 2));
        float x = graph_left + x_step * i;
        float y = 550.0 - output_speed;
        if (y < 50.0)
        {
          y = 50.0;
        }
        if (y > 550.0)
        {
          y = 550.0;
        }
        label.setString("Velocità media = vector_of_mean_speeds[i]");
        point.setPosition((float)x, (float)y);
        window2.draw(point);
      }

      for (auto first = vector_of_mean_distances.begin(),
                last = vector_of_mean_distances.end();
           first != last && ii < visible_count2; ++first, ++ii)
      {
        float x = graph_left + x_step * ii;
        float y = 550.0 - *first;
        if (y < 50.0)
        {
          y = 50.0;
        }
        if (y > 550.0)
        {
          y = 550.0;
        }
        label2.setString("Distanza media = *first ");
        dot.setPosition(x, y);
        window2.draw(dot);
      }
      window2.draw(box);
      window2.draw(label);
      window2.draw(label2);
      window2.display();
    }
  }
}
