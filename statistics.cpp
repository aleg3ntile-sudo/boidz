#include "stats.cpp"

using namespace boidz;

int main()
{
    

    sf::RenderWindow window(sf::VideoMode(800, 600), "Statistics");
    window.setFramerateLimit(60);

    sf::VertexArray velocity_graphic(sf::PrimitiveType::Lines, 4);
    sf::Color axis_color = sf::Color::White;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }

            window.draw(velocity_graphic);
            window.display();
        }
        return 0;
    }
}