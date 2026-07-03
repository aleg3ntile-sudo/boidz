#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <SFML/Graphics.hpp>

namespace boidz
{
    void create_axis(sf::VertexArray &axis);
    void build_label(sf::Text &label, const sf::Font &font);
    void draw_points(std::vector<float> &v, std::vector<float> &w,
                     sf::Text &label, sf::CircleShape &point, sf::RenderWindow &window);
}

#endif