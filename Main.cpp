#include <iostream>
#include <vector>
#include <random>
#include <eigen3/Eigen/Dense>
#include <SFML/Graphics.hpp>

constexpr int win_size = 1000;
constexpr float win_size_2 = static_cast<float>(win_size) / 2;

using vec2 = Eigen::Vector2f;

// Solar system parameters
constexpr float G = 2.0f;  // Gravitational constant
constexpr float M_Sun = 1000;
constexpr float M_Earth = 20;
constexpr float AU = 0.7;  // Astronomical unit

struct Body {
    vec2 pos;
    vec2 vel;
    float mass;
};

enum LagrangePoint {L1, L2, L3, L4, L5};

sf::Vector2f GlToPixelCoord(const vec2& v) {
    return sf::Vector2f(v.x() * win_size / 2 + static_cast<float>(win_size) / 2,
                       -v.y() * win_size / 2 + static_cast<float>(win_size) / 2);
}

sf::Vector2f EigenToSf(const vec2& v) {
    return sf::Vector2f(v.x(), v.y());
}

inline vec2 gravitation(Body& A, Body& B) {
    // Return gravitational force acting on body A
    return G * A.mass * B.mass * (B.pos - A.pos) / (std::pow((B.pos - A.pos).norm(), 3));
}

class Simulation {
public:
    Simulation() {
        // Initialize Sun - Earth system:
        // Calculate earths velocity for circular orbit around sun
        float Earth_velocity = std::sqrt(G * M_Sun / AU);
        
        probes_draw_VA.setPrimitiveType(sf::PrimitiveType::Points);

        addBody({0, 0}, {0, 0}, M_Sun, sf::Color::Yellow);                // Sun
        addBody({AU, 0}, {0, Earth_velocity}, M_Earth, sf::Color::Blue);  // Earth

        // Initialize markers
        orbit_earth_draw.setFillColor(sf::Color::Transparent);
        orbit_earth_draw.setOutlineColor(sf::Color(0, 0, 255, 100));
        orbit_earth_draw.setOutlineThickness(1);
        orbit_earth_draw.setPointCount(100);
        float R_earth = AU * win_size_2;
        orbit_earth_draw.setRadius(R_earth);
        orbit_earth_draw.setOrigin(R_earth, R_earth);
        orbit_earth_draw.setPosition(GlToPixelCoord(vec2(0, 0)));

        // Initialize lagrange point markers
        for (auto& lp : lagrangePoints_draw) {
            lp.setPointCount(15);
            lp.setRadius(10);
            lp.setOrigin(10, 10);
            lp.setFillColor(sf::Color::Transparent);
            lp.setOutlineColor(sf::Color::Red);
            lp.setOutlineThickness(1);
        }
        setLagrangePoint<L1>(bodies[0], bodies[1]);
        setLagrangePoint<L2>(bodies[0], bodies[1]);
        setLagrangePoint<L3>(bodies[0], bodies[1]);
        setLagrangePoint<L4>(bodies[0], bodies[1]);
        setLagrangePoint<L5>(bodies[0], bodies[1]);
    }
    template <LagrangePoint L> void setLagrangePoint(const Body& sun, const Body& earth) {
        vec2 r = earth.pos - sun.pos;
        if constexpr (L == LagrangePoint::L1) { 
            lagrangePoints[L] = sun.pos + r.norm() * (1.0 - std::pow(earth.mass / (3 * (sun.mass + earth.mass)), 0.33333333)) * r.normalized();
        }
        if constexpr (L == LagrangePoint::L2) {
            lagrangePoints[L] = sun.pos + r.norm() * (1.0 + std::pow(earth.mass / (3 * (sun.mass + earth.mass)), 0.33333333)) * r.normalized();
        }
        if constexpr (L == LagrangePoint::L3) { 
            lagrangePoints[L] = sun.pos - r.norm() * (1.0 + 5.0/12.0 * (earth.mass / (earth.mass + sun.mass))) * r.normalized();
        }
        if constexpr (L == LagrangePoint::L4 || L == LagrangePoint::L5) {
            auto sign = L == LagrangePoint::L4 ? 1 : -1;
            constexpr float L45ang = 2.0 * M_PI / 6.0;
            r = vec2(r.x()*cos(L45ang) + sign*r.y()*sin(L45ang), -sign*r.x()*sin(L45ang) + r.y()*cos(L45ang));
            lagrangePoints[L] = r + sun.pos;
        }
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
            bodies[i].vel += dt * body_forces[i] / bodies[i].mass;
            bodies[i].pos += dt * bodies[i].vel;
        }
        
        // Update probe positions and velocities
        for (int i = 0; i < probepointer; ++i) {
            probes[i].vel += dt * probe_forces[i] / probes[i].mass;   // v = v + F/m * t
            probes[i].pos += dt * probes[i].vel;    // p = p + v * t
        }
        // Update lagrange point positions
        setLagrangePoint<L1>(bodies[0], bodies[1]);
        setLagrangePoint<L2>(bodies[0], bodies[1]);
        setLagrangePoint<L3>(bodies[0], bodies[1]);
        setLagrangePoint<L4>(bodies[0], bodies[1]);
        setLagrangePoint<L5>(bodies[0], bodies[1]);
        // bodies[0].pos = vec2(0, 0);
    }

    void addBody(const vec2& pos, const vec2& vel, float mass, sf::Color color=sf::Color::White) {
        constexpr float body_density = 0.1;
        // Calculate radius assuming body is spherical
        float body_radius = std::pow(3.0 * mass / (4.0 * M_PI * body_density), 0.3333333);

        bodies.emplace_back(pos, vel, mass);
        bodies_draw.emplace_back(sf::CircleShape(body_radius));
        bodies_draw.back().setPosition(GlToPixelCoord(bodies.back().pos));
        bodies_draw.back().setOrigin(body_radius, body_radius);
        bodies_draw.back().setFillColor(color);
        body_forces.emplace_back(0, 0);
    }

    void addProbe(const vec2& pos, const vec2& vel, float mass) {
        probes.emplace_back(pos, vel, mass);
        // probes_draw.emplace_back(sf::CircleShape(1));
        // probes_draw.back().setPosition(GlToPixelCoord(probes.back().pos));

        probe_forces.emplace_back(0, 0);
        probecount++;
        probepointer++;
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
                // vec2 pos = vec2(x, y) + refpos;
                // vec2 orb_vel = vec2(-refpos.y(), refpos.x()).normalized();
                // orb_vel *= std::sqrt(G * M_Sun / refpos.norm());

                // addProbe(pos, orb_vel, mass);

                float earth_omega = std::sqrt(G * bodies[0].mass / std::pow(AU, 3));
                vec2 pos = vec2(x, y) + refpos;
                vec2 orb_vel = vec2(-pos.y(), pos.x()).normalized();
                orb_vel *= earth_omega * pos.norm();
                addProbe(pos, orb_vel, mass);
            }
        }
    }

    void addProbeRing(int N, float mass) {
        // Placing N x N probe bodies into the solar system into an orbit around the sun
        std::mt19937 mt_rand(time(0));
        std::uniform_real_distribution<float> phi_dist(0, 2 * M_PI);
        std::uniform_real_distribution<float> r_dist(0.99 * AU, 1.01 * AU);

        for (int i = 0; i < N; ++i) {
            float r   = r_dist(mt_rand);
            float phi = phi_dist(mt_rand);
            
            float x = r * cos(phi);
            float y = r * sin(phi);


            float earth_omega = std::sqrt(G * bodies[0].mass / std::pow(AU, 3));
            vec2 pos = vec2(x, y) + bodies[0].pos;
            vec2 orb_vel = vec2(-pos.y(), pos.x()).normalized();
            orb_vel *= earth_omega * pos.norm();
            addProbe(pos, orb_vel, mass);
        }
    }

    void render(sf::RenderWindow& window) {
        // Update sphere positions
        
        // Draw Earths orbit
        orbit_earth_draw.setPosition(GlToPixelCoord(bodies[0].pos));
        window.draw(orbit_earth_draw);

        // Draw Nbody bodies
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            bodies_draw[i].setPosition(GlToPixelCoord(bodies[i].pos));
        }
        
        // Draw mass probes
        probes_draw_VA.clear();
        for (int i = 0; i < probepointer; ++i) {
            probes_draw_VA.append(sf::Vertex(GlToPixelCoord(probes[i].pos), sf::Color(255, 255, 255, 150)));
        }
        for (const auto& body : bodies_draw) {
            window.draw(body);
        }
        window.draw(probes_draw_VA);

        // Draw lagrange point markers
        for (auto& LP : {L1, L2, L3, L4, L5}) {
            lagrangePoints_draw[LP].setPosition(GlToPixelCoord(lagrangePoints[LP]));
            window.draw(lagrangePoints_draw[LP]);
        }
    }

    void deleteBodies() {
        for (int p = 0; p < probepointer; ++p) {
            vec2 pos = probes[p].pos - bodies[0].pos;
            if (pos.squaredNorm() < std::pow(0.05, 2) || pos.squaredNorm() > std::pow(2, 2)) {
                std::swap(probes[p], probes[probepointer--]);
            }
        }
    }

    float getEarthSunAngle() {
        vec2 temp (bodies[1].pos - bodies[0].pos);
        return atan2(temp.y(), temp.x());
    }

    std::vector<Body> bodies;
    std::vector<Body> probes;

    std::array<vec2, 5> lagrangePoints;
private:
    std::vector<vec2> body_forces;
    std::vector<vec2> probe_forces;
    std::vector<sf::CircleShape> bodies_draw;
    std::vector<sf::CircleShape> probes_draw;
    std::size_t probecount = 0;
    int probepointer = -1;

    sf::CircleShape orbit_earth_draw;
    std::array<sf::CircleShape, 5> lagrangePoints_draw;
    sf::VertexArray probes_draw_VA;
};


int main(int argc, char** argv) {
    sf::ContextSettings context_settings(0, 0, 3, 2, 0);
    sf::RenderWindow window(sf::VideoMode(win_size, win_size), "LagrangePoints");
    window.setFramerateLimit(60);

    Simulation simulation;
    
    if (argc == 2) {
        for (int arg = 1; arg < argc; ++arg) {
            if (argv[arg][0] == 'L') {
                switch (argv[arg][1] - '0') {
                    case 1: simulation.addProbesAround(simulation.lagrangePoints[L1] + vec2(0.008, 0), 0.002, 100, 0.01); break;
                    case 2: simulation.addProbesAround(simulation.lagrangePoints[L2] + vec2(0.0081, 0), 0.001, 100, 0.01); break;
                    case 3: simulation.addProbesAround(simulation.lagrangePoints[L3], 0.001, 50, 0.01); break;
                    case 4: simulation.addProbesAround(simulation.lagrangePoints[L4], 0.003, 50, 0.01); break;
                    case 5: simulation.addProbesAround(simulation.lagrangePoints[L5], 0.003, 50, 0.01); break;
                    default: throw std::runtime_error("Unknown lagrange point");
                }
            }
            else if (argv[arg][0] == 'R') {
                simulation.addProbeRing(10000, 0.01);
            }
        }
    }

    sf::Clock clock;
    sf::View view(sf::Vector2f(win_size_2, win_size_2), sf::Vector2f(win_size, win_size));

    bool delete_bodies = false;
    bool earth_frame = true;

    vec2 view_center = vec2(0, 0);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type) {
                case sf::Event::Closed: window.close(); break;
                case sf::Event::KeyPressed:
                    switch (event.key.code) {
                        case sf::Keyboard::Escape:window.close(); break;
                        case sf::Keyboard::D: delete_bodies = true; break;
                        case sf::Keyboard::R: earth_frame = !earth_frame; break;
                        case sf::Keyboard::Right: view_center += vec2(10, 0); break;
                        case sf::Keyboard::Left: view_center += vec2(-10, 0); break;
                        case sf::Keyboard::Up: view_center += vec2(0, -10); break;
                        case sf::Keyboard::Down: view_center += vec2(0, 10); break;
                        default: break;
                    } break;
                case sf::Event::MouseWheelMoved:
                    view.zoom(event.mouseWheel.delta > 0 ? 0.97 : 1.03);
                    break;
                default : break;
            }
        }
        window.clear(sf::Color::Black);

        // Fix view to first body
        view.setCenter(GlToPixelCoord(simulation.bodies[0].pos));
        // rotate view with earth:
        vec2 r12 = (simulation.bodies[1].pos - simulation.bodies[0].pos).normalized();
        float ang = std::atan2(r12.y(), r12.x());


        if (earth_frame) {
            view.setRotation(-ang * 180.0 / M_PI);
        }
        view.move(EigenToSf(Eigen::Rotation2D(-simulation.getEarthSunAngle()) * view_center));
        window.setView(view);

        if (delete_bodies) {
            simulation.deleteBodies();
        }

        simulation.step(0.0001);
        simulation.render(window);

        window.display();
    }
}
