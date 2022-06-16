#pragma once

#include "CWaggle.h"
#include "EntityControllers.hpp"
#include "SensorTools.hpp"
#include "Config.hpp"
#include "Angles.h"
#include <math.h>
#include <random>

using namespace std;

// Scale the given value from the scale of src to the scale of dst.
double scale(double val, double srcMin, double srcMax, double dstMin, double dstMax) {
    return ((val - srcMin) / (srcMax-srcMin)) * (dstMax-dstMin) + dstMin;
}

int sign(double x) {
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}

enum class State {SEARCH_SPIN, SEARCH_STRAIGHT, FOLLOW_CURVE, ABOUT_FACE, TURN_TO_STRIKE, /*TURN_TO_GOAL, TURN_BY_180,*/ RECOVERY_TURN, DEBUG_STOP};

enum class Chirality {UNKNOWN, CW, CCW};

string toString(State& state)
{
    switch(state) {
        case State::SEARCH_SPIN:        return "SEARCH_SPIN"; break;
        case State::SEARCH_STRAIGHT:    return "SEARCH_STRAIGHT"; break;
        case State::FOLLOW_CURVE:       return "FOLLOW_CURVE"; break;
        case State::ABOUT_FACE:         return "ABOUT_FACE"; break;
        case State::TURN_TO_STRIKE:     return "TURN_TO_STRIKE"; break;
        case State::RECOVERY_TURN:      return "RECOVERY_TURN"; break;
        case State::DEBUG_STOP:         return "DEBUG_STOP"; break;
    }
}

string toString(Chirality& c)
{
    switch(c) {
        case Chirality::UNKNOWN:        return "UNKNOWN"; break;
        case Chirality::CW:             return "CW"; break;
        case Chirality::CCW:            return "CCW"; break;
    }
}


CColor toColor(State& state)
{
    switch(state) {
        case State::SEARCH_SPIN:          return CColor(255, 0, 0, 255); break;
        case State::SEARCH_STRAIGHT:          return CColor(255, 0, 0, 255); break;
        case State::FOLLOW_CURVE:             return CColor(0, 255, 0, 127); break;
        case State::ABOUT_FACE:      return CColor(0, 0, 255, 255); break;
        case State::TURN_TO_STRIKE:       return CColor(255, 255, 255, 127); break;
        case State::RECOVERY_TURN:      return CColor(0, 0, 255, 127); break;
        case State::DEBUG_STOP:         return CColor(255, 255, 255, 127); break;
    }
}

CColor toColor(Chirality& c)
{
    switch(c) {
        case Chirality::UNKNOWN:        return CColor(127, 127, 127, 127); break;
        case Chirality::CW:             return CColor(127, 255, 127, 127); break;
        case Chirality::CCW:            return CColor(255, 127, 127, 127); break;
    }
}

/**
 * Line-following capability with added tail movements to manipulate pucks.
 */
class TailController : public EntityController
{
    std::shared_ptr<World> m_world;
    Entity m_robot;
    default_random_engine &m_rng;
    Config m_config;
    State m_state;
    Chirality m_chirality;
    SensorReading m_reading;

    // Forward and angular speeds.
    double m_v, m_w;

    double m_desiredAngle, m_relativeGoalAngle;

    int m_stepsInState, m_stepsInChirality, m_turnDirection, m_dwellSteps;
    CControllerVis &m_visComponent;

    uniform_int_distribution<> m_searchSpinDist;
    uniform_int_distribution<> m_searchStraightDist;

    double m_puckAngularWidth, m_puckSensorAngularWidth;

    double m_angleDiffThreshold = 0.05;

    CVectorIndicator m_indicator;

public:

    TailController(Entity robot, std::shared_ptr<World> world, default_random_engine &rng, Config &config)
        : m_world(world)
        , m_robot(robot)
        , m_rng(rng)
        , m_config(config)
        , m_chirality(Chirality::UNKNOWN)
        , m_v(0)
        , m_w(0)
        , m_desiredAngle(0)
        , m_stepsInChirality(0)
        , m_turnDirection(1)
        , m_visComponent(m_robot.addComponent<CControllerVis>())
        , m_searchSpinDist(m_config.searchSpinDistMin, m_config.searchSpinDistMax)
        , m_searchStraightDist(m_config.searchStraightDistMin, m_config.searchStraightDistMax)
    {
        setNewState(State::FOLLOW_CURVE);

        m_puckAngularWidth = 2 * atan(m_config.puckRadius / (m_config.robotRadius + m_config.puckRadius));

        // Assuming the PuckSensors (which are circular) are arranged in a circle about
        // the robot.  Thus we can determine this amount---the angular extent of
        // each puck sensor w.r.t. the robot's centre.
        auto& sensors = m_robot.getComponent<CSensorArray>();
        auto &arbitraryPuckSensor = sensors.puckSensorsByType[0][0];
        m_puckSensorAngularWidth = 2 * atan(arbitraryPuckSensor->radius() / arbitraryPuckSensor->distance());
    }

    EntityAction getAction()
    {
        resetIndicator();
/*
if (m_state == State::FOLLOW_CURVE) {
size_t rawValue = (size_t) floor(255 * (m_reading.floorCentre[0]));
double relativeGoalAngle = scale(rawValue, m_config.minCurveIntensity, 255, -M_PI, M_PI);
if (m_chirality == Chirality::CCW)
    relativeGoalAngle = Angles::constrainAngle(relativeGoalAngle + M_PI);
m_indicator.angle = relativeGoalAngle;
m_indicator.length = 500;
m_indicator.r = 255;
m_indicator.g = 0;
m_indicator.b = 0;
m_robot.addComponent<CVectorIndicator>(m_indicator);
}
*/

        SensorTools::ReadSensorArray(m_robot, m_world, m_reading);
        bool C = m_reading.floorCentre[0] > 0 || m_reading.floorCentre[1] > 0 || m_reading.floorCentre[2] > 0;
        bool R = m_reading.floorRight[0] > 0 || m_reading.floorRight[1] > 0 || m_reading.floorRight[2] > 0;
        bool L = m_reading.floorLeft[0] > 0 || m_reading.floorLeft[1] > 0 || m_reading.floorLeft[2] > 0;
        bool onCurve = C && (!R || !L);
        bool offCurve = !C && !R && !L;

        //
        // Handle state transitions...
        //

        // This is reset whenever a new state is set, which should only be done by
        // calling setNewState.
        ++m_stepsInState;

        // This is reset whenever the chirality changes.
        ++m_stepsInChirality;

        if (m_state == State::SEARCH_SPIN) {
            if (onCurve)
                setNewState(State::FOLLOW_CURVE);
            else if (m_stepsInState > m_dwellSteps) {
                setNewState(State::SEARCH_STRAIGHT);
                m_dwellSteps = m_searchStraightDist(m_rng);
            }

        } else if (m_state == State::SEARCH_STRAIGHT) {
            if (onCurve)
                setNewState(State::FOLLOW_CURVE);
            else if (m_reading.robotAhead || m_stepsInState > m_dwellSteps) {
                setNewState(State::SEARCH_SPIN);
                m_dwellSteps = m_searchSpinDist(m_rng);
            }

        } else if (m_state == State::FOLLOW_CURVE) {

            updateChirality();

            // TOO COMPLICATED: Evaluating the swaths for each puck type.  Select the swath with the
            // most pucks.  Swaths containing different puck types are automatically given a value of
            // zero by pucksInSwath.
            std::vector<std::tuple<std::string, int, double>> swaths;
            for (int t=0; t<m_config.numPuckTypes; ++t) {

                // Decode the value in m_reading.floorCentre.
                size_t rawValue = (size_t) floor(255 * (m_reading.floorCentre[t]));

                double relativeGoalAngle = scale(rawValue, m_config.minCurveIntensity, 255, -M_PI, M_PI);
                if (m_chirality == Chirality::CCW)
                    relativeGoalAngle = Angles::constrainAngle(relativeGoalAngle + M_PI);

                int leftSwath = pucksInSwath(t, -M_PI, relativeGoalAngle - m_config.angleThreshold);
                int rightSwath = pucksInSwath(t, relativeGoalAngle + m_config.angleThreshold, M_PI);
                if (leftSwath > 0)
                    swaths.push_back(std::make_tuple("left", leftSwath, relativeGoalAngle - m_config.angleThreshold));
                if (rightSwath > 0)
                    swaths.push_back(std::make_tuple("right", rightSwath, relativeGoalAngle + m_config.angleThreshold));
            }
            auto bestTuple = swaths.begin();
            for (auto tup : swaths)
                if (bestTuple == swaths.begin() || std::get<1>(tup) > std::get<1>(*bestTuple))
                    *bestTuple = tup;
            bool bestTupleValid = bestTuple != swaths.end();
            if (bestTupleValid)
                m_relativeGoalAngle = std::get<2>(*bestTuple);

            // WHAT SHOULD THESE ANGLES BE?
//            double frontSwathPucks = pucksInSwath(0, -M_PI/4, M_PI/4);
//            double maxGoalAngleForStraight = M_PI/2;

            if (offCurve) {
                setNewState(State::SEARCH_SPIN);
                m_dwellSteps = m_searchSpinDist(m_rng);

            } else if (onCurve && m_reading.robotAhead && m_stepsInChirality > 100) {
                forceToggleChirality();
                setNewState(State::ABOUT_FACE);
                m_turnDirection = 1; // This is arbitrary.  Could be -1.
                
//            } else if (onCurve && m_chirality != Chirality::UNKNOWN && bestTupleValid && (frontSwathPucks > 0 && fabs(m_relativeGoalAngle) < maxGoalAngleForStraight)) {
                // Just plow through the pucks ahead, sending them approximately towards their goal.

            } else if (onCurve && m_chirality != Chirality::UNKNOWN && bestTupleValid && m_stepsInState > 100) {//10) {

                // Prepare to strike!
                setNewState(State::TURN_TO_STRIKE);
                if (std::get<0>(*bestTuple) == "left") {
                    m_turnDirection = 1;
                    m_desiredAngle = getCompassAngle() + m_turnDirection * (m_relativeGoalAngle + M_PI);
                } else {
                    m_turnDirection = -1;
                    m_desiredAngle = getCompassAngle() + m_turnDirection * (M_PI - m_relativeGoalAngle);
                }
            }

        } else if (m_state == State::ABOUT_FACE) {
            if (m_stepsInState > m_config.minTurnSteps && C) {
                setNewState(State::FOLLOW_CURVE);
            } else if (m_stepsInState > 100) {
                // We've perhaps been bumped off the curve.  Give up and search.
                setNewState(State::SEARCH_SPIN);
                m_dwellSteps = m_searchSpinDist(m_rng);
            }

        } else if (m_state == State::TURN_TO_STRIKE) {
            if (Angles::getAngularDifference(m_desiredAngle, getCompassAngle()) < m_angleDiffThreshold) {
                setNewState(State::RECOVERY_TURN);
                if (m_turnDirection == 1)
                    m_desiredAngle = getCompassAngle() - (m_relativeGoalAngle + M_PI);
                else
                    m_desiredAngle = getCompassAngle() + (M_PI - m_relativeGoalAngle);

                if (m_config.pucksAsRobots) {
                    // This is to test the idea of being unable to sense the difference between
                    // pucks and robots.  So whenever we recover from the strike, we set the
                    // desired angle as above, but without the addition of pi, leading to this
                    // single expression.
                    m_desiredAngle = getCompassAngle() - m_relativeGoalAngle;
                    forceToggleChirality();
                }
                m_turnDirection *= -1;
            }

        } else if (m_state == State::RECOVERY_TURN) {
            if (Angles::getAngularDifference(m_desiredAngle, getCompassAngle()) < m_angleDiffThreshold) {
                //setNewState(State::DEBUG_STOP);
                setNewState(State::FOLLOW_CURVE);
            }
        }

//if (m_state == State::DEBUG_STOP && m_robot.getComponent<CControllerVis>().selected)
//    setNewState(State::FOLLOW_CURVE);

        // 
        // Knowing what state we are in, act.
        // 

        if (m_state == State::SEARCH_SPIN) {
            m_v = 0;
            m_w = 1;

        } else if (m_state == State::SEARCH_STRAIGHT) {
//            if (m_reading.robotAhead) 
//                m_v = 0;
//            else
                m_v = 1;
            m_w = 0;

        } else if (m_state == State::ABOUT_FACE || m_state == State::TURN_TO_STRIKE || m_state == State::RECOVERY_TURN) {
            // We're in one of the TURN states.
            m_v = 0;
            m_w = m_turnDirection; // Technically, we could do without m_turnDirection
                                   // and just use m_w, but I think it adds clarity.

        } else if (/*m_reading.robotAhead ||*/ m_state == State::DEBUG_STOP) {
            m_v = 0;
            m_w = 0;

        } else if (m_state == State::FOLLOW_CURVE) {
            curveFollow();
        }

        //
        // Debug / visualization
        //

        m_robot.addComponent<CColor>(CColor(50, 50, 255, 255));
        //m_robot.addComponent<CColor>(toColor(m_state));
        //m_robot.addComponent<CColor>(toColor(m_chirality));

//cerr << toString(m_state) << " for " << m_stepsInState << " steps " << endl;
        if (m_robot.getComponent<CControllerVis>().selected) {
            std::stringstream ss;
            ss << m_reading.toString() << endl;
            //ss << "compass angle: " << getCompassAngle() << endl;
            ss << toString(m_state) << " for " << m_stepsInState << " steps " << endl;
            ss << toString(m_chirality) << " for " << m_stepsInChirality << " steps " << endl;
            ss << "v, w: \t" << m_v << ", " << m_w << endl;
            m_visComponent.msg = ss.str();
        }
        m_previousAction = EntityAction(m_v * m_config.maxForwardSpeed, m_w * m_config.maxAngularSpeed);
        return m_previousAction;
    }

    // Swap this later with a more realistic compass simulation.
    double getCompassAngle() {
        auto & steer = m_robot.getComponent<CSteer>();
        return steer.angle;
    }

    void setNewState(State s) {
        m_state = s;
        m_stepsInState = 0;
    }

    void updateChirality() {
        double sensedChiralityMarker = m_reading.floorEdge[2];
        Chirality newChirality = m_chirality;
        if (fabs(sensedChiralityMarker - 1) < 0.1) {
            newChirality = Chirality::CCW;
        } else if (fabs(sensedChiralityMarker - 0.498) < 0.1) {
            newChirality = Chirality::CW;
        }

        // Did we actually change chirality?
        if (newChirality != m_chirality) {
            m_chirality = newChirality;
            m_stepsInChirality = 0;
        }
    }

    void forceToggleChirality() {
        // Now allowing this to be called when the chirality is unknown.  We
        // just allow it to stay unknown.
        // assert(m_chirality != Chirality::UNKNOWN);

        if (m_chirality == Chirality::CW)
            m_chirality = Chirality::CCW;
        else if (m_chirality == Chirality::CCW)
            m_chirality = Chirality::CW;

        m_stepsInChirality = 0;
    }

    // Return the number (approximate number since there is potential overlap between sensors)
    // of pucks of the given type in this angular swath.  If there are pucks of more than one type in the swath
    // return 0.
    int pucksInSwath(size_t puckType, double lowAngle, double highAngle) {
        int total = 0;

        for (int t=0; t<m_config.numPuckTypes; ++t) {
            if (t == puckType)
                total = pucksInSwathHelper(t, lowAngle, highAngle);
            else if (pucksInSwathHelper(t, lowAngle, highAngle) > 0)
                // There is more than one type of puck in this swath.  Return 0 to indicate
                // that it should be ignored.
                return 0;
        }

        return total;
    }

    int pucksInSwathHelper(size_t puckType, double lowAngle, double highAngle) {
        auto& sensors = m_robot.getComponent<CSensorArray>();

        int total = 0;
        for (int i=0; i<m_reading.pucksByType[puckType].size(); ++i) {
            // We're going through the readings, but the angles are stored in the sensors.
            double angle = sensors.puckSensorsByType[puckType][i]->angle();

            if (angle - 0.5 * m_puckSensorAngularWidth >= lowAngle && angle + 0.5 * m_puckSensorAngularWidth <= highAngle)
                total += (int)m_reading.pucksByType[puckType][i];
        }

        return total;
    }


    void curveFollow() {
        bool C = m_reading.floorCentre[0] > 0 || m_reading.floorCentre[1] > 0 || m_reading.floorCentre[2] > 0;
        bool R = m_reading.floorRight[0] > 0 || m_reading.floorRight[1] > 0 || m_reading.floorRight[2] > 0;
        bool L = m_reading.floorLeft[0] > 0 || m_reading.floorLeft[1] > 0 || m_reading.floorLeft[2] > 0;

        int arbitraryTurn = -1;

        m_v = 1;

        if ( !L && !R && !C ) {
            // We're off track.
            m_w = 0;
        } else if (!L && !R && C) {
            // On track, go straight.
            m_w = 0;
        } else if (!L && R && !C) {
            // Track is on the right, go right.
            m_w = 1;
        } else if (!L && R && C) {
            // Track is on the right, go right.
            m_w = 1;
        } else if (L && !R && !C) {
            // Track is on the left, go left.
            m_w = -1;
        } else if (L && !R && C) {
            // Track is on the left, go left.
            m_w = -1;
        } else if (L && R && !C) {
            // This is weird, go straight.
            m_w = 0;
        } else if (L && R && C) {
            // We're facing the line perpendicularly.  Choose the arbitrary turn to join it.
            m_w = arbitraryTurn;
        }
    }

    int getStateAsInt() {
        //return static_cast<std::underlying_type<State>::type>(m_state);
        switch(m_state) {
            case State::SEARCH_SPIN:        return 0; break;
            case State::SEARCH_STRAIGHT:    return 0; break;
            case State::FOLLOW_CURVE:             return 1; break;
            case State::ABOUT_FACE:      return 1; break;
            case State::TURN_TO_STRIKE:       return 1; break;
            case State::RECOVERY_TURN:      return 1; break;
            case State::DEBUG_STOP:         return 1; break;
        }
    }

    void resetIndicator() {
        m_indicator.angle = 0;
        m_indicator.length = 0;
        m_indicator.r = 255;
        m_indicator.g = 255;
        m_indicator.b = 255;
        m_indicator.a = 255;
        m_robot.addComponent<CVectorIndicator>(m_indicator);
    }
};
