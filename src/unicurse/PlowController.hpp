#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "SensorTools.hpp"
#include "Config.hpp"
#include <math.h>
#include <random>

using namespace std;

// Scale the given value from the scale of src to the scale of dst.
double scale(double val, double srcMin, double srcMax, double dstMin, double dstMax) {
    return ((val - srcMin) / (srcMax-srcMin)) * (dstMax-dstMin) + dstMin;
}

enum class State {OFF_CURVE, ON_CURVE, TIMED_TURN};

ostream& operator<< (ostream& out, State& state)
{
    switch(state) {
        case State::OFF_CURVE:      out << "OFF_CURVE"; break;
        case State::ON_CURVE:       out << "ON_CURVE"; break;
        case State::TIMED_TURN:     out << "TIMED_TURN"; break;
    }
    return out;
}

CColor toColor(State& state)
{
    switch(state) {
        case State::OFF_CURVE:      return CColor(127, 127, 127, 127); break;
        case State::ON_CURVE:       return CColor(127, 255, 127, 127); break;
        case State::TIMED_TURN:     return CColor(127, 127, 255, 127); break;
        //case State::STOPPED:    return CColor(227, 227, 227, 127); break;
    }
}

/**
 * Line-following capability with added plow angle modulation.
 */
class PlowController : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity m_robot;
    default_random_engine &m_rng;
    Config m_config;
    State m_state;
    SensorReading m_reading;

    // Forward and angular speeds.
    double m_v, m_w;

    int m_turnCountdown;
    CControllerVis &m_visComponent;

public:

    PlowController(Entity robot, std::shared_ptr<World> world, default_random_engine &rng, Config &config)
        : m_world(world)
        , m_robot(robot)
        , m_rng(rng)
        , m_config(config)
        , m_state(State::OFF_CURVE)
        , m_v(0)
        , m_w(0)
        , m_turnCountdown(0)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
    {
    }

    EntityAction getAction()
    {
        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);

        // Default forward and angular speeds.
        m_v = 1.0;
        m_w = 0.0;

        bool wrongWay = m_reading.gridVec[5] > 0;
        if (wrongWay && m_state == State::ON_CURVE) {
            m_state = State::TIMED_TURN;
            m_turnCountdown = 80;
        }

        if (m_state == State::TIMED_TURN) {
            m_v = 0;
            m_w = -1;
            if (--m_turnCountdown == 0) {
                m_state = State::ON_CURVE;
            }
        } else {
            normalState();
        }

        //
        // Debug / visualization
        //

        m_robot.addComponent<CColor>(toColor(m_state));
        if (m_robot.getComponent<CControllerVis>().selected) {
            std::stringstream ss;
            ss << m_reading.toString() << endl;
            ss << "v, w: \t" << m_v << ", " << m_w << endl;
            m_visComponent.msg = ss.str();
        }
        m_previousAction = EntityAction(m_v * m_config.maxForwardSpeed, m_w * m_config.maxAngularSpeed);
        return m_previousAction;
    }

    void normalState() {
        bool C = m_reading.gridVec[0] > 0;
        bool R = m_reading.gridVec[1] > 0 || m_reading.gridVec[3] > 0;
        bool L = m_reading.gridVec[2] > 0 || m_reading.gridVec[4] > 0;

        int arbitraryTurn = -1;

        if ( !L && !R && !C ) {
            // We're off track.
            m_w = 0;
            m_state = State::OFF_CURVE;
        } else if (!L && !R && C) {
            // On track, go straight.
            m_w = 0;
            m_state = State::ON_CURVE;
        } else if (!L && R && !C) {
            // Track is on the right, go right.
            m_w = 1;
            m_state = State::OFF_CURVE;
        } else if (!L && R && C) {
            // Track is on the right, go right.
            m_w = 1;
            m_state = State::ON_CURVE;
        } else if (L && !R && !C) {
            // Track is on the left, go left.
            m_w = -1;
            m_state = State::OFF_CURVE;
        } else if (L && !R && C) {
            // Track is on the left, go left.
            m_w = -1;
            m_state = State::ON_CURVE;
        } else if (L && R && !C) {
            // This is weird, go straight.
            m_w = 0;
            m_state = State::OFF_CURVE;
        } else if (L && R && C) {
            // We're facing the line perpendicularly.  Choose the arbitrary turn to join it.
            m_w = arbitraryTurn;
            m_robot.addComponent<CColor>(50, 255, 50, 255);
            m_state = State::OFF_CURVE;
        }

        double plowAngle = M_PI / 4.0;

        // This has to match the lowest value used in the scalar field image for part of the curve.
        double minIntensity = 0.2;

        if (m_state == State::ON_CURVE) {
            // ONLY FLIP PLOW ANGLE IF GETTING A VALID CENTRE READING.
            double decisionValue = scale(m_reading.gridVec[0], minIntensity, 1, -M_PI, M_PI);
            if (decisionValue > 0)
                m_robot.getComponent<CPlowBody>().angle = plowAngle;
            else
                m_robot.getComponent<CPlowBody>().angle = -plowAngle;
        }
    }

    int getStateAsInt() {
        return static_cast<std::underlying_type<State>::type>(m_state);
    }
};
