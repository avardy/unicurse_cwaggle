#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "SensorTools.hpp"
#include "Config.hpp"
#include <math.h>
#include <random>

using namespace std;

/**
 * This controller is intended to be used to compare with a Q-Learning solution
 * (hash function GridArc3).
 */
class CompatibleController : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity m_robot;
    default_random_engine &m_rng;
    Config m_config;
    CControllerVis &m_visComponent;

public:

    CompatibleController(Entity robot, std::shared_ptr<World> world, default_random_engine &rng, Config &config)
        : m_world(world)
        , m_robot(robot)
        , m_rng(rng)
        , m_config(config)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
    {
    }

    EntityAction getAction()
    {
        SensorReading reading;
        SensorTools::ReadSensorArray(m_robot, m_world, reading);

        // Forward and angular speeds.
        double v = 1.0;
        double w = 0.0;

        bool C = reading.gridVec[0] > 0;
        bool R = reading.gridVec[1] > 0;
        bool L = reading.gridVec[2] > 0;

        int arbitraryTurn = 1;

        if ( !L && !R && !C ) {
            // We're off track.
            w = arbitraryTurn;
            m_robot.addComponent<CColor>(50, 50, 50, 255);
        } else if (!L && !R && C) {
            // On track, go straight.
            w = 0;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (!L && R && !C) {
            // Track is on the right, go right.
            w = 1;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (!L && R && C) {
            // Track is on the right, go right.
            w = 1;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (L && !R && !C) {
            // Track is on the left, go left.
            w = -1;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (L && !R && C) {
            // Track is on the left, go left.
            w = -1;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (L && R && !C) {
            // This is weird, go straight.
            w = 0;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        } else if (L && R && C) {
            // We're facing the line perpendicularly.  Choose the arbitrary turn to join it.
            w = arbitraryTurn;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
        }

        //
        // Debug / visualization
        //

        if (m_robot.getComponent<CControllerVis>().selected) {
            std::stringstream ss;
            ss << reading.toString() << endl;
            ss << "v, w: \t" << v << ", " << w << endl;
            m_visComponent.msg = ss.str();
        }
        m_previousAction = EntityAction(v * m_config.maxForwardSpeed, w * m_config.maxAngularSpeed);
        return m_previousAction;
    }

    // Placeholder
    int getStateAsInt() {
        return 0;
    }
};
