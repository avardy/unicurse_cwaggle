#pragma once

#include <float.h>
#include <string>
using std::string;

#include "Components.hpp"
#include "Entity.hpp"
#include "World.hpp"

class Sensor {
protected:
    size_t m_ownerID; // entity that owns this sensor
    string m_name;
    double m_angle = 0; // angle sensor is placed w.r.t. owner heading
    double m_distance = 0; // distance from center of owner

public:
    Sensor() { }
    Sensor(size_t ownerID, string name, double angle, double distance)
        : m_ownerID(ownerID)
        , m_name(name)
        , m_angle(angle * M_PI / 180.0)
        , m_distance(distance)
    {
    }

    inline string name() const
    {
        return m_name;
    }

    inline virtual Vec2 getPosition()
    {
        const Vec2& pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = m_angle + Entity(m_ownerID).getComponent<CSteer>().angle;
        return pos + Vec2(m_distance * cos(sumAngle), m_distance * sin(sumAngle));
    }

    inline virtual double angle() const
    {
        return m_angle;
    }

    inline virtual double distance() const
    {
        return m_distance;
    }

    virtual double getReading(std::shared_ptr<World> world) = 0;
};

class GridSensor : public Sensor {
public:
    size_t m_gridIndex;

    GridSensor(size_t ownerID, string name, size_t gridIndex, double angle, double distance)
        : Sensor(ownerID, name, angle, distance)
    {
        m_gridIndex = gridIndex;
    }

    inline virtual double getReading(std::shared_ptr<World> world)
    {
        if (world->getGrid(m_gridIndex).width() == 0) {
            return 0;
        }
        Vec2 sPos = getPosition();
        size_t gX = (size_t)round(sPos.x);
        size_t gY = (size_t)round(sPos.y);
        return world->getGrid(m_gridIndex).get(gX, gY);
    }
};


class PuckSensor : public Sensor
{
    std::string m_typeName;
    size_t m_puckType;
    double m_radius;

public:

    PuckSensor(size_t ownerID, string name, std::string typeName, size_t puckType, double angle, double distance, double radius)
        : Sensor(ownerID, name, angle, distance)
    {
        m_typeName = typeName;
        m_puckType = puckType;
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto puck : world->getEntities(m_typeName))
        {
            if (puck.getComponent<CPuckType>().type != m_puckType) { continue; }

            auto & t = puck.getComponent<CTransform>();
            auto & b = puck.getComponent<CCircleBody>();

            // collision with a puck
            if (t.p.distSq(pos) < (m_radius + b.r)*(m_radius + b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }

    inline void adjustAngle(double deltaAngle)
    {
        m_angle += deltaAngle;
    }
};


// Detects collisions with CLineBody objects.
class ObstacleSensor : public Sensor
{
    double m_radius;

public:

    ObstacleSensor(size_t ownerID, string name, double angle, double distance, double radius)
        : Sensor(ownerID, name, angle, distance)
    {
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto e : world->getEntities())
        {
            if (!e.hasComponent<CLineBody>()) { continue; }
            if (m_ownerID == e.id()) { continue; }

            auto & b = e.getComponent<CLineBody>();

            double lineX1 = b.e.x - b.s.x;
            double lineY1 = b.e.y - b.s.y;
            double lineX2 = pos.x - b.s.x;
            double lineY2 = pos.y - b.s.y;

            double edgeLength = lineX1 * lineX1 + lineY1 * lineY1;
            double dotProd = lineX1 * lineX2 + lineY1 * lineY2;
            double t = std::max(0.0, std::min(edgeLength, dotProd)) / edgeLength;

            // find the closest point on the line to the sensor and the distance to it
            Vec2 closestPoint(b.s.x + t * lineX1, b.s.y + t * lineY1);
            double distance = closestPoint.dist(pos);

            // pretend the closest point on the line is a circle and check collision
            // calculate the overlap between the circle and that fake circle
            double overlap = m_radius + b.r - distance;
            if (overlap > 0) {
                sum += 1.0;                
            } 
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }
};

class RobotSensor : public Sensor
{
public:
    double m_radius;

    RobotSensor(size_t ownerID, string name, double angle, double distance, double radius)
        : Sensor(ownerID, name, angle, distance)
    {
        m_radius = radius;
    }

    inline double getReading(std::shared_ptr<World> world)
    {
        double sum = 0;
        Vec2 pos = getPosition();
        for (auto e : world->getEntities())
        {
            if (!e.hasComponent<CSteer>()) { continue; }
            if (m_ownerID == e.id()) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // collision with other robot
            if (t.p.distSq(pos) < (m_radius + b.r)*(m_radius + b.r))
            {
                sum += 1.0;
            }
        }
        return sum;
    }

    inline double radius() const
    {
        return m_radius;
    }
};


// Used by FancySensor below.  Represents a circle that a target object should either be within or without.
class SensingCircle {
public:
    double m_angle = 0;
    double m_distance = 0;
    double m_radius = 0;
    bool m_within = true;
    double m_distanceOffset = 0;

    SensingCircle(double a, double d, double r, bool within, double offset=0) 
        : m_angle(a)
        , m_distance(d)
        , m_radius(r)
        , m_within(within)
        , m_distanceOffset(offset)
    { }  
};

// A fancy-shaped sensor.  Considering the sets of points within (or without) the given vector of SensingCirlces.
class FancySensor
{
protected:

    size_t m_ownerID;       // entity that owns this sensor
public:
    std::string m_typeName;
    std::string m_name;
    std::vector<SensingCircle> m_circles;
    bool m_mustTouchLeft = false;
    bool m_mustTouchRight = false;
    double m_leftCentreThreshold, m_rightCentreThreshold;
protected:
    bool m_senseMaxGridValue = false;
    int m_gridIndex = 0;

public:
    FancySensor() {}
    FancySensor(size_t ownerID, std::string typeName, std::string name,
                std::vector<SensingCircle> inCircles, bool mustTouchLeft, bool mustTouchRight,
                double leftCentreThreshold=0, double rightCentreThreshold=0)
        : m_ownerID(ownerID)
        , m_typeName(typeName)
        , m_name(name)
        , m_circles(inCircles)
        , m_mustTouchLeft(mustTouchLeft)
        , m_mustTouchRight(mustTouchRight)
        , m_leftCentreThreshold(leftCentreThreshold)
        , m_rightCentreThreshold(rightCentreThreshold)
    { }

    inline int getNumberOfCircles()
    {
        return m_circles.size();
    }

    inline double getCircleRadius(int index)
    {
        return m_circles[index].m_radius;
    }

    inline Vec2 getCirclePosition(int index, std::shared_ptr<World> world)
    {
        double distance = m_circles[index].m_distance + m_circles[index].m_distanceOffset;

        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double sumAngle = Entity(m_ownerID).getComponent<CSteer>().angle + m_circles[index].m_angle;
        return pos + Vec2(distance * cos(sumAngle), distance * sin(sumAngle));
    }

    inline double getFancyReading(std::shared_ptr<World> world, size_t puckType)
    {
        double returnValue = 0;

        std::vector<Vec2> posVector;
        for (int i=0; i<m_circles.size(); i++)
            posVector.push_back(getCirclePosition(i, world));

        // Position and angle of robot.  Needed only if m_mustBeLeft || m_mustBeRight.
        const Vec2 & pos = Entity(m_ownerID).getComponent<CTransform>().p;
        double theta = Entity(m_ownerID).getComponent<CSteer>().angle;

        for (auto e : world->getEntities(m_typeName))
        {
            if (e.id() == m_ownerID) { continue; }
            if (e.getComponent<CPuckType>().type != puckType) { continue; }

            auto & t = e.getComponent<CTransform>();
            auto & b = e.getComponent<CCircleBody>();

            // Go through all sensing circles and determine whether the current position passes
            // the test of being within/without that circle (according to the circle's m_within).
            // Note that for testing to be without a circle, we test against a circle whose radius 
            // is reduced by one diameter of the sensed object.
            bool objectSensed = true;
            for (int i=0; i<m_circles.size(); i++) {
                SensingCircle c = m_circles[i];
                double radius = c.m_radius;

                if ((c.m_within && t.p.distSq(posVector[i]) > (radius + b.r)*(radius + b.r))
                    ||
                    (!c.m_within && t.p.distSq(posVector[i]) < (radius - b.r)*(radius - b.r)))
                {
                    objectSensed = false;
                    continue;
                }
            }

            if (m_mustTouchLeft || m_mustTouchRight) {
                // Y-coordinate w.r.t. robot ref. frame
                double y = -sin(theta) * (t.p.x - pos.x) + cos(theta) * (t.p.y - pos.y);
                if (m_mustTouchLeft && y - b.r > m_leftCentreThreshold)
                    objectSensed = false;
                if (m_mustTouchRight && y + b.r < -m_rightCentreThreshold)
                    objectSensed = false;
            }

            if (objectSensed) {
                returnValue += 1.0;
                //std::cout << t.p.x << "_" << t.p.y << std::endl;
                //e.addComponent<CColor>(0, 0, rand()%255, 255);
            }
        }
        return returnValue;
    }
};