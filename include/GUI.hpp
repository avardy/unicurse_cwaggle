#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#if _WIN32
#   include <Windows.h>
#endif
#if __APPLE__
#   include <OpenGL/glu.h>
#else
#   include <GL/glu.h>
#endif

#include <iostream>
#include <sstream>

#include "Simulator.hpp"
#include "Vec2.hpp"
#include "SensorTools.hpp"
#include "KeyboardCallback.hpp"

class GUI {
    std::shared_ptr<Simulator> m_sim;
    sf::RenderWindow m_window; // the window we will draw to
    sf::Font m_font; // the font we will use to draw
    sf::Text m_text;
    sf::Clock m_clock;
    Vec2 m_mousePos;
    Entity m_draggedEntity;
    bool m_debug = false;
    bool m_sensors = false;
    bool m_drawLines = true, m_drawCircles = true, m_zoom = false;
//    size_t m_controlsHeight = 250;
size_t m_controlsHeight = 0;
    size_t m_windowWidth;
    size_t m_windowHeight;
    size_t m_windowScale;
    double m_zoomFactor = 5.0;

private:
    std::string m_status = "";
    bool m_leftMouseDown = false;

    // AV: For drawing a background image.
    sf::Texture m_backgroundTexture;
    sf::Sprite m_backgroundSprite;

    // A vector of pairs of (image, bool) where the bool indicates whether the
    // image should be drawn. 
    std::vector<std::pair<sf::Image, bool>> m_backgroundImages;

    // AV: For showing the occupancy of robots
    sf::Image m_occupancyImage;

    KeyboardCallback* m_keyboardCallback = nullptr;

    void init(std::shared_ptr<Simulator> sim)
    {
        m_sim = sim;
        m_font.loadFromFile("fonts/cour.ttf");
        m_text.setFont(m_font);
        m_text.setCharacterSize(24);
        m_text.setPosition(5, 5);
        //m_text.setFillColor(sf::Color::Yellow);

        int width = m_sim->getWorld()->width();
        int height = m_sim->getWorld()->height();

        // Create all images which can be used as the background.
        for (int i = 0; i < m_sim->getWorld()->getNumberOfGrids(); i++) {
            auto& grid = m_sim->getWorld()->getGrid(i);
            assert(grid.width() == width);
            assert(grid.height() == height);

            sf::Image gridImage;
            gridImage.create(width, height);

            for (size_t x = 0; x < grid.width(); x++) {
                for (size_t y = 0; y < grid.height(); y++) {
                    uint8_t c = (uint8_t)(grid.get(x, y) * 255);
                    sf::Color color(c, c, c);
                    gridImage.setPixel(x, y, color);
                }
            }

            m_backgroundImages.push_back(std::make_pair(gridImage, i == 0));
        }

        m_occupancyImage.create(width, height);

        m_backgroundTexture.loadFromImage(m_occupancyImage);
        m_backgroundSprite.setTexture(m_backgroundTexture);
    }

    void rotateRobots(double angle)
    {
        for (auto& entity : m_sim->getWorld()->getEntities("robot")) {
            if (entity.hasComponent<CControllerVis>() && entity.getComponent<CControllerVis>().selected) {
                auto& steer = entity.getComponent<CSteer>();
                steer.angle += angle;
            }
        }
    }

    void userInput()
    {
        sf::Event event;
        while (m_window.pollEvent(event)) {
            // this event triggers when the window is closed
            if (event.type == sf::Event::Closed) {
                exit(0);
            }

            // this event is triggered when a key is pressed
            if (event.type == sf::Event::KeyPressed) {

                if (m_keyboardCallback != nullptr)
                    m_keyboardCallback->keyHandler(event.key.code);

                switch (event.key.code) {
                case sf::Keyboard::Escape:
                    exit(0);
                    break;
                case sf::Keyboard::C:
                    m_drawCircles = !m_drawCircles;
                    break;
                case sf::Keyboard::D:
                    m_debug = !m_debug;
                    break;
                case sf::Keyboard::S:
                    m_sensors = !m_sensors;
                    break;
                case sf::Keyboard::L:
                    m_drawLines = !m_drawLines;
                    break;
                //case sf::Keyboard::O:
                //    m_backgroundImagePtr = &m_occupancyImage;
                //    break;
                case sf::Keyboard::Num0:
                    m_backgroundImages[0].second = !m_backgroundImages[0].second;
                    break;
                case sf::Keyboard::Num1:
                    m_backgroundImages[1].second = !m_backgroundImages[1].second;
                    break;
                case sf::Keyboard::Num2:
                    m_backgroundImages[2].second = !m_backgroundImages[2].second;
                    break;
                case sf::Keyboard::Num3:
                    m_backgroundImages[3].second = !m_backgroundImages[3].second;
                    break;
                case sf::Keyboard::Num4:
                    m_backgroundImages[4].second = !m_backgroundImages[4].second;
                    break;
                case sf::Keyboard::Num5:
                    m_backgroundImages[5].second = !m_backgroundImages[5].second;
                    break;
                //case sf::Keyboard::Num6:
                //    m_backgroundImages[6].second = !m_backgroundImages[6].second;
                //    break;
                case sf::Keyboard::Left:
                    rotateRobots(-0.15);
                    break;
                case sf::Keyboard::Right:
                    rotateRobots(0.15);
                    break;
                case sf::Keyboard::A:
                    // Select all robots
                    for (auto e : m_sim->getWorld()->getEntities()) {
                        if (!e.hasComponent<CControllerVis>()) { continue; }
                        e.getComponent<CControllerVis>().selected = true;
                    }
                    break;

                case sf::Keyboard::N:
                    // De-select all robots
                    for (auto e : m_sim->getWorld()->getEntities()) {
                        if (!e.hasComponent<CControllerVis>()) { continue; }
                        e.getComponent<CControllerVis>().selected = false;
                    }
                    break;
                case sf::Keyboard::Z:
                    m_zoom = !m_zoom;
                    break;

                default:
                    break;
                }
            }

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    for (auto e : m_sim->getWorld()->getEntities()) {
                        //Vec2 mPos((double)event.mouseButton.x, (double)event.mouseButton.y);
                        Vec2 mPos = transformMouse(event.mouseButton.x, event.mouseButton.y);
                        if (mPos.dist(e.getComponent<CTransform>().p) < e.getComponent<CCircleBody>().r) {
                            m_draggedEntity = e;
                            break;
                        }
                    }
                }

                // Right-click modifies an entity's ControllerVis object (if it has one)
                if (event.mouseButton.button == sf::Mouse::Right) {
                    for (auto e : m_sim->getWorld()->getEntities()) {
                        if (!e.hasComponent<CControllerVis>()) { continue; }

                        //Vec2 mPos((double)event.mouseButton.x, (double)event.mouseButton.y);
                        Vec2 mPos = transformMouse(event.mouseButton.x, event.mouseButton.y);
                        if (mPos.dist(e.getComponent<CTransform>().p) < e.getComponent<CCircleBody>().r) {
                            // Toggle the selected status.
                            e.getComponent<CControllerVis>().selected = 
                                !(e.getComponent<CControllerVis>().selected);
                            break;
                        }
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    m_draggedEntity = Entity();
                }
            }

            if (event.type == sf::Event::MouseMoved) {
                //m_mousePos = sf::Vector2f((float)event.mouseMove.x, (float)event.mouseMove.y);
		m_mousePos = transformMouse(event.mouseMove.x, event.mouseMove.y);
            }
        }

        if (m_draggedEntity != Entity()) {
            auto& t = m_draggedEntity.getComponent<CTransform>();
            /*
            Vec2 diff(m_mousePos.x - t.p.x, m_mousePos.y - t.p.y);
            diff /= 10;
            t.v = diff;
            */
            t.p.x = m_mousePos.x;
            t.p.y = m_mousePos.y;
        }
    }

    void drawLine(Vec2 p1, Vec2 p2, sf::Color color)
    {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f((float)p1.x, (float)p1.y), color),
            sf::Vertex(sf::Vector2f((float)p2.x, (float)p2.y), color)
        };

        m_window.draw(line, 2, sf::Lines);
    }

    void render()
    {
        m_window.clear();

        // Fill the occupancy grid
        sf::Color color(255, 255, 255);
        for (auto robot : m_sim->getWorld()->getEntities("robot")) {
            auto& t = robot.getComponent<CTransform>();
            m_occupancyImage.setPixel((int)t.p.x, (int)t.p.y, color);
        }

        // Draw all active background images, which are blended together
        for (std::pair<sf::Image, bool> p : m_backgroundImages) {
            if (p.second) {
                m_backgroundTexture.update(p.first);
                m_window.draw(m_backgroundSprite, sf::RenderStates(sf::BlendAdd));
            }
        }

        // draw robot plows
        for (auto e : m_sim->getWorld()->getEntities()) {
            if (!e.hasComponent<CPlowBody>()) {
                continue;
            }

            auto& t = e.getComponent<CTransform>();
            auto& pb = e.getComponent<CPlowBody>();
            auto& c = e.getComponent<CColor>();
            auto& steer = e.getComponent<CSteer>();

            pb.shape.setPosition((float)t.p.x, (float)t.p.y);
            pb.shape.setRotation((steer.angle + pb.angle) * 180.0 / M_PI);
            pb.shape.setFillColor(sf::Color(c.r, c.g, c.b, c.a));
            m_window.draw(pb.shape);

            // Draw a line along the prow.
            /*
            Vec2 prow(t.p.x + pb.length * cos(steer.angle + pb.angle),
                      t.p.y + pb.length * sin(steer.angle + pb.angle));
            drawLine(t.p, prow, sf::Color(255, 255, 255));
            */
        }

        // draw attached line bodies
        for (auto e : m_sim->getWorld()->getEntities()) {
            if (!e.hasComponent<CAttachedLineBody>()) {
                continue;
            }

            auto& t = e.getComponent<CTransform>();
            auto& alb = e.getComponent<CAttachedLineBody>();
            auto& c = e.getComponent<CColor>();
            auto& steer = e.getComponent<CSteer>();

            // Transform the start and end points into the world ref. frame.
            Vec2 start(t.p.x + alb.s.x * cos(steer.angle) - alb.s.y * sin(steer.angle),
                       t.p.y + alb.s.x * sin(steer.angle) + alb.s.y * cos(steer.angle));
            Vec2 end(t.p.x + alb.e.x * cos(steer.angle) - alb.e.y * sin(steer.angle),
                       t.p.y + alb.e.x * sin(steer.angle) + alb.e.y * cos(steer.angle));
            drawLine(start, end, sf::Color(255, 255, 255));
        }

        // draw circles
        if (m_drawCircles) {
            for (auto e : m_sim->getWorld()->getEntities()) {
                if (!e.hasComponent<CCircleShape>()) {
                    continue;
                }

                auto& t = e.getComponent<CTransform>();
                auto& s = e.getComponent<CCircleShape>();
                auto& c = e.getComponent<CColor>();

                s.shape.setPosition((float)t.p.x, (float)t.p.y);
                s.shape.setFillColor(sf::Color(c.r, c.g, c.b, c.a));

                m_window.draw(s.shape);

                // Highlight frozen or slowed robots.
/*
                if (e.hasComponent<CSteer>()) {
                    auto& steer = e.getComponent<CSteer>();
                    if (steer.frozen || steer.slowedCount > 0) {
                        float l = s.shape.getRadius();
                        sf::RectangleShape shape(sf::Vector2f(l, l));
                        if (steer.frozen)
                            shape.setFillColor(sf::Color(50, 50, 50));
                        else
                            shape.setFillColor(sf::Color(255, 0, 255));

                        shape.setPosition((float)t.p.x - l/2, (float)t.p.y - l/2);
                        m_window.draw(shape);
                    }
                }
*/

                // Draw a line corresponding to this circle's velocity.
                Vec2 velPoint;
                double vLength = t.v.length();
                if (vLength == 0) {
                    velPoint = Vec2(t.p.x + s.shape.getRadius(), t.p.y);
                    continue;
                } else {
                    velPoint = t.p + t.v.normalize() * s.shape.getRadius();
                }

                drawLine(t.p, velPoint, sf::Color(255, 255, 255));
            }
        }

        // draw robot sensors
        if (m_sensors)
        {
            float gridSensorRadius = 2;
            for (auto robot : m_sim->getWorld()->getEntities("robot"))
            {
                // if (!m_draggedEntity || robot.id() != m_draggedEntity.id()) { continue; }

                if (!robot.hasComponent<CSensorArray>()) { continue; }
                auto & sensors = robot.getComponent<CSensorArray>();
                auto & c = robot.getComponent<CColor>();

                // Position and angle of robot.
                const Vec2 & pos = robot.getComponent<CTransform>().p;
                double theta = robot.getComponent<CSteer>().angle;

                int numberPuckTypes = sensors.puckSensorsByType.size();

                for (auto sensor : sensors.fancySensors)
                {
                    sf::Color detectColor(255, 0, 255, 50);
                    sf::Color noDetectColor(255, 0, 127, 50);
                    for (int t=0; t<numberPuckTypes; ++t) {
                        double reading = sensor->getFancyReading(m_sim->getWorld(), t);
                        for (int i=0; i<sensor->getNumberOfCircles(); i++) {

                            sf::CircleShape cShape((float)sensor->getCircleRadius(i), 132);
                            if (sensor->m_typeName == "puck")
                                cShape.setFillColor(sf::Color(255, 0, 0, 0)); 
                            if (sensor->m_typeName == "robot")
                                cShape.setFillColor(sf::Color(0, 0, 255, 0)); 

                            cShape.setOrigin((float)sensor->getCircleRadius(i), (float)sensor->getCircleRadius(i));
                            Vec2 pos = sensor->getCirclePosition(i, m_sim->getWorld());
                            cShape.setPosition((float)pos.x, (float)pos.y);

                            if (reading > 0) { 
                                cShape.setFillColor(detectColor);
                                cShape.setOutlineThickness(1);
                            } else { 
                                cShape.setFillColor(noDetectColor);
                                cShape.setOutlineThickness(0);
                            }
                            m_window.draw(cShape);
                        }
                    }
                }
                for (int t=0; t<numberPuckTypes; ++t) {
                    for (auto & sensor : sensors.puckSensorsByType[t]) {
                        sf::Color detectColor(255, 0, 255, 100);
                        sf::Color noDetectColor(255, 0, 127, 100);
                        // BAD: Loop contents same as below.
                        sf::CircleShape sensorShape(sensor->radius(), 32);
                        sensorShape.setOrigin(sensor->radius(), sensor->radius());
                        Vec2 pos = sensor->getPosition();
                        sensorShape.setPosition((float)pos.x, (float)pos.y);
                        double reading = sensor->getReading(m_sim->getWorld());
                        sensorShape.setOutlineColor(sf::Color::White);
                        if (reading > 0) {
                            sensorShape.setFillColor(detectColor);
                            sensorShape.setOutlineThickness(1);
                        } else {
                            sensorShape.setFillColor(noDetectColor);
                            sensorShape.setOutlineThickness(0);
                        }
                        m_window.draw(sensorShape);
                    }
                }
                for (auto & sensor : sensors.robotSensors) {
                    sf::Color detectColor(0, 0, 255, 200);
                    sf::Color noDetectColor(0, 0, 127, 200);
                    // BAD: Loop contents same as above.
                    sf::CircleShape sensorShape(sensor->radius(), 32);
                    sensorShape.setOrigin(sensor->radius(), sensor->radius());
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    double reading = sensor->getReading(m_sim->getWorld());
                    sensorShape.setOutlineColor(sf::Color::White);
                    if (reading > 0) {
                        sensorShape.setFillColor(detectColor);
                        sensorShape.setOutlineThickness(1);
                    } else {
                        sensorShape.setFillColor(noDetectColor);
                        sensorShape.setOutlineThickness(0);
                    }
                    m_window.draw(sensorShape);
                }
                for (auto & sensor : sensors.gridSensors)
                {
                    sf::CircleShape sensorShape(gridSensorRadius, 32);
                    sensorShape.setOrigin(gridSensorRadius, gridSensorRadius);
                    Vec2 pos = sensor->getPosition();
                    sensorShape.setPosition((float)pos.x, (float)pos.y);
                    sensorShape.setFillColor(sf::Color::White);
                    m_window.draw(sensorShape);
                }
            }
        }

        // Draw other robot-specific "decorations".
        for (auto robot : m_sim->getWorld()->getEntities("robot")) {
            auto& t = robot.getComponent<CTransform>();
            auto& s = robot.getComponent<CCircleShape>();
            auto& c = robot.getComponent<CColor>();
            auto& steer = robot.getComponent<CSteer>();

            // Draw a line corresponding to this robot's heading.
            double r = s.shape.getRadius();
            Vec2 start(t.p.x, t.p.y);
            Vec2 end(t.p.x + r * cos(steer.angle), t.p.y + r * sin(steer.angle));
            drawLine(start, end, sf::Color(0, 0, 0));

            // If the robot is selected, draw an outline around it.
            if (robot.hasComponent<CControllerVis>() && robot.getComponent<CControllerVis>().selected) {
                /*
                r *= 2;
                sf::Vertex vertices[] = {
                    sf::Vertex(sf::Vector2f(t.p.x - r/2, t.p.y - r/2), sf::Color::White),
                    sf::Vertex(sf::Vector2f(t.p.x + r/2, t.p.y - r/2), sf::Color::White),
                    sf::Vertex(sf::Vector2f(t.p.x + r/2, t.p.y + r/2), sf::Color::White),
                    sf::Vertex(sf::Vector2f(t.p.x - r/2, t.p.y + r/2), sf::Color::White),
                    sf::Vertex(sf::Vector2f(t.p.x - r/2, t.p.y - r/2), sf::Color::White)
                };
                m_window.draw(vertices, 5, sf::LineStrip);
                */
                auto& t = robot.getComponent<CTransform>();
                auto& s = robot.getComponent<CCircleShape>();
                s.shape.setPosition((float)t.p.x, (float)t.p.y);
                s.shape.setFillColor(sf::Color(0, 0, 0, 0));
                s.shape.setOutlineColor(sf::Color(255, 255, 255, 255));
                s.shape.setOutlineThickness(1);
                m_window.draw(s.shape);
                s.shape.setOutlineThickness(0);
            }

            if (robot.hasComponent<CTerritory>()) {
                auto& territory = robot.getComponent<CTerritory>();
                territory.shape.setRadius((float)territory.radius);
                territory.shape.setOrigin((float)territory.radius, (float)territory.radius);
                territory.shape.setPosition((float)territory.centre.x, (float)territory.centre.y);
                territory.shape.setFillColor(sf::Color(0, 0, 0, 0));
                territory.shape.setOutlineColor(territory.color);
                territory.shape.setOutlineThickness(1);
                m_window.draw(territory.shape);
            }
        }

        // Draw CVectorIndicator objects, which could be attached to any entity
        for (auto e : m_sim->getWorld()->getEntities()) {
            if (e.hasComponent<CVectorIndicator>()) {
                auto& vi = e.getComponent<CVectorIndicator>();
                auto& t = e.getComponent<CTransform>();

                double angle = vi.angle;
                if (e.hasComponent<CVectorIndicator>()) {
                    auto& steer = e.getComponent<CSteer>();
                    angle += steer.angle;
                }

                Vec2 start(t.p.x, t.p.y);
                Vec2 end(t.p.x + vi.length * cos(angle),
                         t.p.y + vi.length * sin(angle));
                drawLine(start, end, sf::Color(vi.r, vi.g, vi.b, vi.a));
            }
        }

        if (m_drawLines) {
            sf::Color lineColor(200, 200, 200);
            for (auto& e : m_sim->getWorld()->getEntities("line")) {
                auto& line = e.getComponent<CLineBody>();

                sf::CircleShape circle((float)line.r, 32);
                circle.setFillColor(lineColor);
                circle.setOrigin((float)line.r, (float)line.r);
                circle.setOutlineColor(lineColor);
                circle.setOutlineThickness(1);
                circle.setPosition((float)line.s.x, (float)line.s.y);
                m_window.draw(circle);
                circle.setPosition((float)line.e.x, (float)line.e.y);
                m_window.draw(circle);

                Vec2 normal(-(line.e.y - line.s.y), (line.e.x - line.s.x));
                normal = normal.normalize() * line.r;

                drawLine(line.s + normal, line.e + normal, lineColor);
                drawLine(line.s - normal, line.e - normal, lineColor);
            }
        }

        if (m_debug) {
            for (auto& collision : m_sim->getCollisions()) {
                drawLine(collision.t1->p, collision.t2->p, sf::Color::Green);
            }
        }
    }

    void renderControls() {
        // Draw "controls" area at the bottom of the screen.
        sf::RectangleShape rect(sf::Vector2f(m_windowWidth, m_controlsHeight));
        rect.setPosition(0, m_sim->getWorld()->height());
        rect.setFillColor(sf::Color(100, 100, 100, 255));
        m_window.draw(rect);

        // Draw the status text
        sf::Text text;
        text.setFont(m_font);
        text.setString(m_status);
        text.setCharacterSize(12);
        text.setPosition(5, (float)m_sim->getWorld()->height());// + text.getLocalBounds().height);
        m_window.draw(text);
    }

public:
    GUI(std::shared_ptr<Simulator> sim, size_t fps)
        : m_sim(sim)
	, m_windowScale(2)
    {
        m_windowWidth = m_sim->getWorld()->width();
        m_windowHeight = m_sim->getWorld()->height() + m_controlsHeight;
        m_window.create(sf::VideoMode(m_windowWidth, m_windowHeight), "CWaggle", sf::Style::Titlebar | sf::Style::Close);
        m_window.setFramerateLimit(fps);

        // Scale the window size up for high-res screens.
        size_t scaledWindowWidth = m_windowScale * m_windowWidth;
        size_t scaledWindowHeight = m_windowScale * m_windowHeight;
        m_window.setSize(sf::Vector2u(scaledWindowWidth, scaledWindowHeight));
        init(sim);
    }

    void setStatus(const std::string& str)
    {
        m_status = str;
    }

    void setSim(std::shared_ptr<Simulator> sim)
    {
        init(sim);
    }

    Vec2 transformMouse(double mouseX, double mouseY) {
    	double x = mouseX / m_windowScale;
    	double y = mouseY / m_windowScale;

    	if (m_zoom)
            std::cerr << "Mouse not supported while zoomed-in." << std::endl;

    	return Vec2(x, y);
    }

    // Get the position of the first of the set of selected robots.  If no robots
    // are selected, return the centre position.
    Vec2 getSelectedRobotPos() {
        Vec2 pos(m_windowWidth/2, m_windowHeight/2);
        for (auto& robot : m_sim->getWorld()->getEntities("robot")) {
            if (robot.hasComponent<CControllerVis>() && robot.getComponent<CControllerVis>().selected) {
                auto& transform = robot.getComponent<CTransform>();
                pos.x = transform.p.x;
                pos.y = transform.p.y;
                break;
            }
        }
        return pos;
    }

    void update()
    {
        userInput();
        render();

        size_t width = m_windowScale * m_windowWidth;
        size_t height = m_windowScale * m_windowHeight;
        if (m_zoom) {
            // Zoom to centre the first of the selected robots 
            Vec2 pos = getSelectedRobotPos();

            double x = - m_zoomFactor * m_windowScale * pos.x;
            double y = m_zoomFactor * m_windowScale * (pos.y - m_windowHeight);
            glViewport(x + width/2, y + height/2, width*m_zoomFactor, height*m_zoomFactor);
        } else {
            //glViewport(0, 0, m_windowWidth, m_windowHeight);
            glViewport(0, 0, width, height);
        }

        renderControls();

        m_window.display();
    }

    void close()
    {
        m_window.close();
    }

    void updateGridImage(int gridIndex, bool red, bool green, bool blue)
    {
        // Get a copy of the grid and normalize it.
        ValueGrid grid{m_sim->getWorld()->getGrid(gridIndex)};
        grid.normalize();

        int w = grid.width();
        int h = grid.height();

        auto& gridImage = m_backgroundImages[gridIndex].first;

        for (size_t x = 0; x < w; x++) {
            for (size_t y = 0; y < h; y++) {
                uint8_t value = (uint8_t)(grid.get(x, y) * 255);
                uint8_t r=0, g=0, b=0;
                if (red)
                    r = value;
                if (green)
                    g = value;
                if (blue)
                    b = value;
                sf::Color color(r, g, b);
                gridImage.setPixel(x, y, color);
            }
        }
    }

    void setKeyboardCallback(KeyboardCallback* keyboardCallback)
    {
        m_keyboardCallback = keyboardCallback;
    }

    void saveScreenshot(std::string filename)
    {
        //sf::Image screenshot = m_window.capture();
        //screenshot.saveToFile(filename);
        sf::Texture texture;
        texture.create(m_window.getSize().x, m_window.getSize().y);
        texture.update(m_window);
        texture.copyToImage().saveToFile(filename);
    }
};
