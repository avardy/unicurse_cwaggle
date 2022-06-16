#pragma once

#include <memory>
#include <fstream>
#include <string>
#include <functional>
#include <numeric>

#include "CWaggle.h"
#include "GUI.hpp"
#include "QLearning.hpp"
#include "Eval.hpp"
#include "Hash.hpp"
#include "Config.hpp"
#include "UnicurseWorld.hpp"
#include "SpeedManager.hpp"
#include "DataLogger.hpp"
//#include "PlowController.hpp"
#include "TailController.hpp"

class RLExperiment
{
    Config          m_config;
    int             m_trialIndex;

    default_random_engine       m_rng;
    SpeedManager                m_speedManager;
    DataLogger                  m_dataLogger;

    QLearning                   m_QL;

    std::shared_ptr<GUI>        m_gui;
    std::shared_ptr<Simulator>  m_sim;
    std::shared_ptr<World>      m_world;

    std::vector<Entity>         m_robotsActed;
    std::vector<size_t>         m_states;
    std::vector<size_t>         m_actions;
    std::vector<size_t>         m_nextStates;
    std::vector<double>         m_rewards;
    size_t                      m_stepsUntilRLUpdate = 1;
    size_t                      m_episodeStartTime = 0;

    double                      m_simulationTime = 0;
    Timer                       m_simTimer;

    double                      m_totalReward, m_eval, m_propSlowed, m_cumEval = 0;
    std::stringstream           m_status;

public:

    RLExperiment(const Config & config, int trialIndex, int rngSeed)
        : m_config(config)
        , m_trialIndex(trialIndex)
        , m_rng(rngSeed)
        , m_speedManager(config)
        , m_dataLogger(config, trialIndex)
    {
        m_QL = QLearning(m_config.numStates, m_config.numActions, m_config.alpha, m_config.gamma, m_config.initialQ);

        if (m_config.loadQ)
            m_QL.load(m_config.loadQFile);

        if (m_config.loadQPartial)
            m_QL.loadPartial(m_config.loadQFilePartial);

        resetSimulator();
        m_stepsUntilRLUpdate = m_config.batchSize;
    }

    void doSimulationStep()
    {
        // Update the robot occupancy grid by filling in pixels at the position of grid sensor 0.
        ValueGrid &occGrid = m_world->getGrid(3);
        /*
        for (auto robot : m_world->getEntities("robot")) {
            auto & sensors = robot.getComponent<CSensorArray>();
            auto & gridSensor0 = sensors.gridSensors[0];
            Vec2 pos = gridSensor0->getPosition();
            size_t gX = (size_t)round(pos.x);
            size_t gY = (size_t)round(pos.y);
            occGrid.set(gX, gY, 1);
        }
        */

        SensorReading reading;

        // control robots that have controllers
        for (auto robot : m_world->getEntities("robot")) {
            // record the robot sensor state into the batch
            SensorTools::ReadSensorArray(robot, m_world, reading);
            m_states.push_back(m_config.hashFunction(reading));

            // Record reward per robot
            double reward = 0;
            if (m_config.penaltyDotRadius == 0) {
                if (reading.floorCentre[0] > 0)
                    reward = 1;
            } else {
                auto & sensors = robot.getComponent<CSensorArray>();
                auto & gridSensor0 = sensors.gridSensors[0];
                Vec2 pos = gridSensor0->getPosition();
                size_t gX = (size_t)round(pos.x);
                size_t gY = (size_t)round(pos.y);
                // Give a reward only if the position of sensor 0 has not already been marked as occupied
                bool giveReward = reading.floorCentre[0] > 0 && occGrid.get(gX, gY) == 0;
                if (giveReward) {
                    reward = 1;
                    occGrid.setAllWithinRadius(gX, gY, m_config.penaltyDotRadius, 1.0);
                }
            }

            m_rewards.push_back(reward);

            // The index of the chosen action.
            size_t actionIndex = 0;

            // get the action that should be done for this entity
            EntityAction action;

            if (m_config.manual) {
                action = robot.getComponent<CController>().controller->getAction();
            } else {
                // epsilon-greedy action selection
                if ((rand() / (double)RAND_MAX) < m_config.epsilon) {
                    // cerr << "RANDOM ACTION" << endl;
                    actionIndex = rand() % m_config.numActions;
                    action = getAction(actionIndex);
                } else {
                    actionIndex = m_QL.selectActionFromPolicy(m_config.hashFunction(reading));
                    action = getAction(actionIndex);
                }
            }

            // record the action that the robot did into the batch
            //m_actions.push_back(getActionIndex(action));
            m_actions.push_back(actionIndex);
            m_robotsActed.push_back(robot);

            // have the action apply its effects to the entity
            action.doAction(robot, m_speedManager.getSimTimeStep());
        }

        // call the world physics simulation update
        // parameter = how much sim time should pass (default 1.0)
        if (m_speedManager.getSimTimeStep() > 0)
            m_sim->update(m_speedManager.getSimTimeStep());

        // record the robot next states to the batch
        for (auto & robot : m_world->getEntities("robot")) {
            // record the robot sensor state into the batch
            SensorTools::ReadSensorArray(robot, m_world, reading);
            m_nextStates.push_back(m_config.hashFunction(reading));
        }

        if (m_states.size() != m_actions.size() || m_states.size() != m_nextStates.size())
            std::cout << "Warning: Batch Size Mismatch: S " << m_states.size() << " A " << m_actions.size() << " NS " << m_nextStates.size() << "\n";

        // Evaluate overall performance
        m_totalReward = accumulate(m_rewards.begin(), m_rewards.end(), 0);
        m_eval = 0;
        // BAD: Accept a variable number of goals.
        if (m_config.numPuckTypes >= 1)
            //m_eval += Eval::PuckSSDFromIdealPosition(m_world, 0, Vec2{m_config.redGoalX, m_config.redGoalY});
            m_eval += Eval::PuckGridValues(m_world, 0, 4);
        if (m_config.numPuckTypes >= 2)
            //m_eval += Eval::PuckSSDFromIdealPosition(m_world, 1, Vec2{m_config.greenGoalX, m_config.greenGoalY});
            m_eval += Eval::PuckGridValues(m_world, 1, 5);
        m_eval /= m_config.numPuckTypes;
        m_propSlowed = Eval::ProportionSlowedRobots(m_world);
        m_cumEval += m_eval;

        // if the batch size has been reached, do the update
        if (--m_stepsUntilRLUpdate == 0) {

            if (m_config.updateQTable) {
                for (size_t i = 0; i < m_states.size(); i++) {
                    m_QL.updateValue(m_states[i], m_actions[i], m_rewards[i], m_nextStates[i]);
                    m_QL.updatePolicy(m_states[i]);
                }
            }

            // clean up the data structures for next batch
            m_states.clear();
            m_rewards.clear();
            m_actions.clear();
            m_nextStates.clear();
            m_robotsActed.clear();
            m_stepsUntilRLUpdate = m_config.batchSize;
        }

        int n = m_speedManager.getStepCount();
        if (m_config.writeDataSkip && n % m_config.writeDataSkip == 0)
            m_dataLogger.writeToFile(m_world, n, m_eval, m_propSlowed, m_cumEval, m_QL.getCoverage());

        if (m_config.saveQSkip && n % m_config.saveQSkip == 0)
            m_QL.save(m_config.saveQFile);

        if (m_config.saveQSkipPartial && n % m_config.saveQSkipPartial == 0)
            m_QL.savePartial(m_config.saveQFilePartial);

        m_speedManager.incrementStepCount();
    }

    void run() {
        bool running = true;
        while (running) {
            m_simTimer.start();
            for (size_t i = 0; i < m_speedManager.getRenderSteps(); i++) {
                if (m_config.maxTimeSteps > 0 && m_speedManager.getStepCount() >= m_config.maxTimeSteps) {
                    running = false;
                }
                doSimulationStep();
            }
            m_simulationTime += m_simTimer.getElapsedTimeInMilliSec();

            m_status = stringstream();
            m_status << "Step: " << m_speedManager.getStepCount() << endl;
            m_status << "Eval: " << m_eval << endl;
            m_status << "Prop Slowed: " << m_propSlowed << endl;
            //m_status << "QO Coverage: " << m_QL.getCoverage() << " of " << m_QL.size() << "\n";
            //m_status << "Cum Eval: " << m_cumEval << endl;
            //m_status << "Cum Eval / Step: " << (m_cumEval / (m_speedManager.getStepCount() - m_episodeStartTime)) << endl;

            if (!m_gui && (m_speedManager.getStepCount() % 100000 == 0))
                std::cout << m_status.str();

            if (m_gui) {
                /*
                m_gui->updateGridImage(0, true, false, false);
                m_gui->updateGridImage(1, false, true, false);
                m_gui->updateGridImage(2, false, false, true);
                m_gui->updateGridImage(3, false, false, true);
                if (m_config.numPuckTypes >= 1)
                    m_gui->updateGridImage(4, true, false, false);
                if (m_config.numPuckTypes >= 2)
                    m_gui->updateGridImage(5, false, true, false);
                */

                for (auto robot : m_world->getEntities("robot")) {
                    if (!robot.hasComponent<CControllerVis>()) { continue; }
                    auto vis = robot.getComponent<CControllerVis>();

                    if (vis.selected) {
                        //SensorReading reading;
                        //SensorTools::ReadSensorArray(robot, m_world, reading);
                        //vis.msg = reading.toString();
                        m_status << vis.msg;
                    }
                }

                m_gui->setStatus(m_status.str());

                // draw gui
                m_gui->update();

                if (m_config.captureScreenshots) {
                    stringstream filename;
                    filename << m_config.screenshotFilenameBase << m_speedManager.getStepCount() / m_speedManager.getRenderSteps() << ".png";
                    m_gui->saveScreenshot(filename.str());
                }
            }

            // End the episode.
            if (m_config.episodeTimeSteps > 0 && (m_speedManager.getStepCount() - m_episodeStartTime) > m_config.episodeTimeSteps) {
                //m_speedManager.resetStepCount();
                resetSimulator();
                m_cumEval = 0;
                m_episodeStartTime = m_speedManager.getStepCount();
                // Reset the grid used to track occupancy
                m_world->getGrid(3).setAll(0);
            }
        }
    }

    double getEval() {
        return m_eval;
    }

private:

    void resetSimulator()
    {
        if (m_world == NULL)
            m_world = unicurse_world::GetWorld(m_rng, m_config);
        else
            unicurse_world::randomizeWorld(m_world, m_rng, m_config);

        m_sim = make_shared<Simulator>(m_world);

        if (m_gui) {
            m_gui->setSim(m_sim);
        } else if (m_config.gui) {
            m_gui = make_shared<GUI>(m_sim, 144);
            m_gui->setKeyboardCallback(&m_speedManager);
        }

        if (m_gui) {
            m_gui->updateGridImage(0, true, true, true);
            m_gui->updateGridImage(1, false, true, false);
            m_gui->updateGridImage(2, false, false, true);
            m_gui->updateGridImage(3, false, false, true);
            if (m_config.numPuckTypes >= 1)
                m_gui->updateGridImage(4, true, false, false);
            if (m_config.numPuckTypes >= 2)
                m_gui->updateGridImage(5, false, true, false);
        }

        if (m_config.manual)
            for (auto e : m_world->getEntities("robot"))
//                e.addComponent<CController>(make_shared<PlowController>(e, m_world, m_rng, m_config));
                e.addComponent<CController>(make_shared<TailController>(e, m_world, m_rng, m_config));
    }
    
    EntityAction getAction(size_t actionIndex)
    {
        return EntityAction(m_config.maxForwardSpeed, m_config.maxAngularSpeed * m_config.actions[actionIndex]);
    }

};