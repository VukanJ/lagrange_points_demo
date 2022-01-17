#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <SFML/Graphics.hpp>

constexpr int win_size = 800;
constexpr float win_size_2 = static_cast<float>(win_size) / 2;

using vec2 = Eigen::Vector2f;

sf::Vector2f GlToPixelCoord(const vec2& v) {
    return sf::Vector2f(v.x() * win_size / 2 + static_cast<float>(win_size) / 2,
                       -v.y() * win_size / 2 + static_cast<float>(win_size) / 2);
}

struct Body {
    vec2 pos;
    vec2 vel;
    float mass;
};

constexpr float G = 1.0f;  // Gravitational constant
constexpr float M_Sun = 1000;
constexpr float M_Earth = 15;
constexpr float AU = 0.7;  // Astronomical unit


inline vec2 gravitation(Body& A, Body& B) {
    // Return gravitational force acting on body A
    return G * A.mass * B.mass * (B.pos - A.pos) / (std::pow((B.pos - A.pos).norm(), 3));
}

class Simulation {
public:
    Simulation(std::size_t N) {
        // Initialize Sun - Earth system

        // Calculate velocity for circular orbit
        float Earth_velocity = std::sqrt(G * M_Sun / AU);
        
        probes_draw_VA.setPrimitiveType(sf::PrimitiveType::Points);

        addBody({0, 0}, {0, 0}, M_Sun, sf::Color::Yellow);                // Sun
        addBody({AU, 0}, {0, Earth_velocity}, M_Earth, sf::Color::Blue);  // Earth
    }

    void integrateForces() {
        // Reset forces
        std::fill(body_forces.begin(), body_forces.end(), vec2(0, 0));
        std::fill(probe_forces.begin(), probe_forces.end(), vec2(0, 0));
        
        // Compute gravitational attraction (2 - body problem)
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                auto Fg = gravitation(bodies[i], bodies[j]);
                // Newton 3
                body_forces[i] += Fg;
                body_forces[j] -= Fg;
            }
        }
        
        // Forces acting on probes
        for (size_t i = 0; i < probes.size(); ++i) {
            auto Fg = gravitation(probes[i], bodies[0]) + gravitation(probes[i], bodies[1]);
            probe_forces[i] += Fg;
        }
    }

    void step(float dt) {
        integrateForces();
        for (size_t i = 0; i < bodies.size(); ++i) {
            bodies[i].vel += dt * body_forces[i] / bodies[i].mass;   // v = v + F/m * t
            bodies[i].pos += dt * bodies[i].vel;    // p = p + v * t
        }
        // Update probe positions and velocities
        for (size_t i = 0; i < probes.size(); ++i) {
            probes[i].vel += dt * probe_forces[i] / probes[i].mass;   // v = v + F/m * t
            probes[i].pos += dt * probes[i].vel;    // p = p + v * t
        }
    }

    void addBody(const vec2& pos, const vec2& vel, float mass, sf::Color color=sf::Color::White) {
        constexpr float body_density = 0.1;

        bodies.emplace_back(pos, vel, mass);
        bodies_draw.emplace_back(sf::CircleShape(std::pow(3.0 * mass / (4.0 * M_PI * body_density), 0.3333333)));
        bodies_draw.back().setPosition(GlToPixelCoord(bodies.back().pos));
        bodies_draw.back().setFillColor(color);
        body_forces.emplace_back(0, 0);
    }

    void addProbe(const vec2& pos, const vec2& vel, float mass) {
        probes.emplace_back(pos, vel, mass);
        // probes_draw.emplace_back(sf::CircleShape(1));
        // probes_draw.back().setPosition(GlToPixelCoord(probes.back().pos));

        probe_forces.emplace_back(0, 0);
    }

    void addProbes(int N, float mass) {
        // Placing N x N probe bodies into the solar system into an orbit around the sun
        constexpr float min = -0.9;
        constexpr float max =  0.9001;
        for (float x = min; x < max; x += (max - min) / N) {
            for (float y = min; y < max; y += (max - min) / N) {
                vec2 pos(x, y);
                vec2 orb_vel = vec2(-y, x).normalized();
                orb_vel *= std::sqrt(G * M_Sun / pos.norm());

                addProbe(pos, orb_vel, mass);
            }
        }
    }

    void addProbesAround(vec2 refpos, float width, int N, float mass) {
        // Placing N x N probe bodies into the solar system into an orbit around the sun
        const float min = -0.9 * width;
        const float max =  0.9001 * width;
        for (float x = min; x < max; x += (max - min) / N) {
            for (float y = min; y < max; y += (max - min) / N) {
                vec2 pos = vec2(x, y) + refpos;
                vec2 orb_vel = vec2(-pos.y(), pos.x()).normalized();
                orb_vel *= std::sqrt(G * M_Sun / refpos.norm());

                addProbe(pos, orb_vel, mass);
            }
        }
    }

    void render(sf::RenderWindow& window) {
        // Update sphere positions
        for (int i = 0; i < bodies.size(); ++i) {
            bodies_draw[i].setPosition(GlToPixelCoord(bodies[i].pos));
        }
        probes_draw_VA.clear();
        for (int i = 0; i < probes.size(); ++i) {
            probes_draw_VA.append(sf::Vertex(GlToPixelCoord(probes[i].pos), sf::Color(255, 255, 255, 150)));
        }
        // Draw everything
        for (const auto& body : bodies_draw) {
            window.draw(body);
        }
        window.draw(probes_draw_VA);
    }

    std::vector<Body> bodies;
    std::vector<Body> probes;

    std::vector<sf::CircleShape> bodies_draw;
    std::vector<sf::CircleShape> probes_draw;
    sf::VertexArray probes_draw_VA;
private:
    std::vector<vec2> body_forces;
    std::vector<vec2> probe_forces;
};


int main() {
    sf::ContextSettings context_settings(0, 0, 3, 2, 0);
    sf::RenderWindow window(sf::VideoMode(800, 800), "LagrangePoints");
    window.setFramerateLimit(60);

    Simulation simulation(10);
    // simulation.addProbes(100, 0.1);
    simulation.addProbesAround(vec2(0.5, 0.5), 0.3, 100, 0.1);
    simulation.addProbesAround(vec2(0.5, -0.5), 0.3, 100, 0.1);

    sf::Clock clock;
    sf::View view(sf::Vector2f(win_size_2, win_size_2), sf::Vector2f(win_size, win_size));

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type) {
                case sf::Event::Closed: window.close(); break;
                case sf::Event::KeyPressed:
                    switch (event.key.code) {
                        case sf::Keyboard::Escape:window.close(); break;
                        default: break;
                    } break;
                default : break;
            }
        }
        window.clear(sf::Color::Black);

        // Fix view to first body
        view.setCenter(GlToPixelCoord(simulation.bodies[0].pos));
        // rotate view with earth:
        vec2 r12 = (simulation.bodies[1].pos - simulation.bodies[0].pos).normalized();
        float ang = std::atan2(r12.y(), r12.x());
        view.setRotation(-ang * 180.0 / M_PI);

        window.setView(view);
        

        simulation.step(0.001);
        simulation.render(window);
        window.display();
    }
}
