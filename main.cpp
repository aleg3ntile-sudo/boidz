#include "rules.hpp"
#include "stats.hpp"
#include "graphics.hpp"
#include <algorithm>
#include <random>

using namespace boidz;
int main()
{
  int flock_size{};
  Parameters params{};
  try
  {
    params.d = read_parameter("Neighbours distance ", 20.f, 100.f);
    params.d_s = read_parameter("Critical distance ", 5.f, 25.f);
    params.s = read_parameter("Separation parameter ", 0.5f, 5.f);
    params.a = read_parameter("Alignment parameter ", 0.05f, 1.f);
    params.c = read_parameter("Cohesion parameter ", 0.01f, 0.05f);
    flock_size = read_parameter("Flock size ", 1, 200);
    params.h = 1.;
    params.p = 0.5;
  }
  catch (const std::runtime_error &e)
  {
    std::cerr << "Errore" << '\n';
    return EXIT_FAILURE;
  }

  Flock stormo{};
  stormo.generate_boids(flock_size);

  Boid hunter{{400., 300.}, {0., 0.}, -1};
  hunter.set_color(sf::Color(255, 100, 100));

  sf::RenderWindow window(sf::VideoMode(800, 600), "Boids");
  window.setFramerateLimit(60);
  sf::RenderWindow window2(sf::VideoMode(500, 600), "Velocity");
  window2.setFramerateLimit(60);
  window2.setPosition(sf::Vector2i(12, 300));
  sf::RenderWindow window3(sf::VideoMode(500, 600), "Distance");
  window3.setFramerateLimit(60);
  window3.setPosition(sf::Vector2i(1422, 300));

  sf::Clock clock;

  sf::CircleShape point(2.f);
  point.setFillColor(sf::Color::Red);
  sf::CircleShape dot(2.f);
  dot.setFillColor(sf::Color::Green);

  sf::VertexArray axis(sf::PrimitiveType::Lines, 4);
  create_axis(axis);

  sf::Text label;
  sf::Text label2;
  sf::Font font;
  font.loadFromFile("font.ttf");
  build_label(label, font);
  build_label(label2, font);

  const size_t max_points = 450;
  Statistics stats;

  float timer = 0.;
  const float delay = 5.;

  while (window.isOpen() || window2.isOpen() || window3.isOpen())
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
    while (window3.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
      {
        window3.close();
      }
    }

    if (window.isOpen())
    {
      float dt = clock.restart().asSeconds();
      timer += dt;
      std::vector<Coords> updated_velocities(stormo.flock.size());
      for (size_t i = 0; i < stormo.flock.size(); ++i)
      {
        updated_velocities[i] = create_velocity_with_rules(params, stormo,
                                                           stormo.flock[i], hunter);
      }

      for (size_t i = 0; i < stormo.flock.size(); ++i)
      {
        stormo.flock[i].update_velocity_with_rules(updated_velocities[i]);
        stormo.flock[i].update_position_in_time(dt);
        stormo.flock[i].border_restriction(800., 600.);
        stormo.flock[i].speed_restriction(130);
        stormo.flock[i].update_shape();
      }

      if (timer >= delay)
      {
        Coords new_hunt_velocity = hunt_the_flock(hunter, stormo, params) +
                                   hunt_neighbours(hunter, stormo, params);
        hunter.update_velocity_with_rules(new_hunt_velocity);
        hunter.update_position_in_time(dt);
        hunter.border_restriction(800., 600.);
        hunter.speed_restriction(100);
        hunter.update_shape();
      }

      float mean_speed_for_cycle = mean_speed(stormo);
      stats.vector_of_mean_speeds.push_back(mean_speed_for_cycle);
      if (stats.vector_of_mean_speeds.size() > max_points)
      {
        stats.vector_of_mean_speeds.erase(
            stats.vector_of_mean_speeds.begin(),
            stats.vector_of_mean_speeds.begin() +
                static_cast<long int>(
                    (stats.vector_of_mean_speeds.size() - max_points)));
      }
      auto speeds = get_vector_of_speeds(stormo);
      float standard_deviation_speed = standard_deviation(speeds);
      stats.vector_of_standard_deviations_speed.push_back(standard_deviation_speed);

      float mean_distance_boidz = mean_distance(stormo);
      stats.vector_of_mean_distances.push_back(mean_distance_boidz);
      auto vector_of_distances = get_vector_of_distances(stormo);
      float standard_deviation_distance =
          standard_deviation(vector_of_distances);
      stats.vector_of_standard_deviations_distance.push_back(standard_deviation_distance);
      if (stats.vector_of_mean_distances.size() > max_points)
      {
        stats.vector_of_mean_distances.erase(
            stats.vector_of_mean_distances.begin(),
            stats.vector_of_mean_distances.begin() +
                static_cast<long int>(
                    (stats.vector_of_mean_distances.size() -
                     max_points))); // toglie i punti extra dal grafico
      }
      if (stats.vector_of_standard_deviations_speed.size() > max_points)
      {
        stats.vector_of_standard_deviations_speed.erase(
            stats.vector_of_standard_deviations_speed.begin(),
            stats.vector_of_standard_deviations_speed.begin() +
                static_cast<long int>(
                    (stats.vector_of_standard_deviations_speed.size() -
                     max_points)));
      }
      if (stats.vector_of_standard_deviations_distance.size() > max_points)
      {
        stats.vector_of_standard_deviations_distance.erase(
            stats.vector_of_standard_deviations_distance.begin(),
            stats.vector_of_standard_deviations_distance.begin() +
                static_cast<long int>(
                    (stats.vector_of_standard_deviations_distance.size() -
                     max_points)));
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

      auto visible_count = stats.vector_of_mean_speeds.size();
      float graph_left = 50.0;
      float graph_right = 450.0;
      float graph_width = graph_right - graph_left;
      float x_step;
      if (visible_count > 1)
      {
        x_step = graph_width / static_cast<float>((visible_count - 1));
      }
      else
      {
        x_step = 0.0;
      }

      for (long unsigned int i{0}; i < visible_count; ++i)
      {
        float output_speed = stats.vector_of_mean_speeds[i];
        float x = graph_left + (x_step * static_cast<float>(i));
        float y = 550.0f - output_speed;
        if (y < 50.0f)
        {
          y = 50.0f;
        }
        if (y > 550.0f)
        {
          y = 550.0f;
        }
        label.setString(
            "Mean speed = " + std::to_string(output_speed) + '\n' +
            "Error = " + std::to_string(stats.vector_of_standard_deviations_speed[i]));
        point.setPosition(x, y);
        window2.draw(point);
      }
      window2.draw(label);
      window2.display();
    }

    if (window3.isOpen())
    {
      window3.clear();
      window3.draw(axis);

      auto visible_count2 = stats.vector_of_mean_distances.size();
      long unsigned int i{0};
      float graph_left = 50.0;
      float graph_right = 450.0;
      float graph_width = graph_right - graph_left;
      float x_step;
      if (visible_count2 > 1)
      {
        x_step = graph_width / static_cast<float>((visible_count2 - 1));
      }
      else
      {
        x_step = 0.0;
      }
      for (auto first = stats.vector_of_mean_distances.begin(),
                last = stats.vector_of_mean_distances.end();
           first != last && i < visible_count2; ++first, ++i)
      {
        float x = graph_left + (x_step * static_cast<float>(i));
        float y = 550.0f - *first;
        if (y < 50.0f)
        {
          y = 50.0f;
        }
        if (y > 550.0f)
        {
          y = 550.0f;
        }
        label2.setString(
            "Mean distance =" + std::to_string(*first) + '\n' +
            "Error = " + std::to_string(stats.vector_of_standard_deviations_distance[i]));
        dot.setPosition(x, y);
        window3.draw(dot);
      }
      window3.draw(label2);
      window3.display();
    }
  }
}
