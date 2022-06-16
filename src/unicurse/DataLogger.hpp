#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <iostream>

// For mkdir
#include <sys/stat.h>
#include <sys/types.h>

#include "CWaggle.h"

#include "Config.hpp"
#include "TailController.hpp"

using namespace std;

class DataLogger {
    Config m_config;
    int m_trialIndex;
    ofstream m_statsStream, m_robotPoseStream, m_robotStateStream;

    // There is an output stream for each puck type.
    vector<shared_ptr<ofstream>> m_puckPositionStreamVector;

public:
    DataLogger(const Config & config, int trialIndex)
        : m_config(config)
        , m_trialIndex(trialIndex)
    {
        //cout << "rngSeed: " << rngSeed << endl;
        if (m_config.writeDataSkip) {

            if (mkdir(config.dataFilenameBase.c_str(), 0777) == -1 && errno != EEXIST)
                cerr << "Error creating directory: " << m_config.dataFilenameBase << endl;

            stringstream statsFilename, robotPoseFilename, robotStateFilename;
            statsFilename << m_config.dataFilenameBase << "/stats_" << trialIndex << ".dat";
            robotPoseFilename << m_config.dataFilenameBase << "/robotPose_" << trialIndex << ".dat";
            robotStateFilename << m_config.dataFilenameBase << "/robotState_" << trialIndex << ".dat";

            m_statsStream = ofstream(statsFilename.str());
            m_robotPoseStream = ofstream(robotPoseFilename.str());
            m_robotStateStream = ofstream(robotStateFilename.str());

            for (int puckType=0; puckType<m_config.numPuckTypes; ++puckType) {
                stringstream puckPositionFilename;
                puckPositionFilename << m_config.dataFilenameBase << "/puckType_" << puckType << "_" << trialIndex << ".dat";
                m_puckPositionStreamVector.push_back(make_shared<ofstream>(puckPositionFilename.str()));
            }

        }
    }

    void writeToFile(shared_ptr<World> world, double stepCount, double eval, double propSlowed, double cumEval, double qTableCoverage)
    {
        double avgState = 0;
        double n = 0;
        for (auto& robot : world->getEntities("robot")) {
            // Ugly!
            std::shared_ptr<TailController> ctrl = std::dynamic_pointer_cast<TailController>(robot.getComponent<CController>().controller);
            avgState += ctrl->getStateAsInt();
            ++n;
        }
        if (n > 0) {
            avgState /= n;
        }

        m_statsStream << stepCount << " " << eval << " " << propSlowed << " " << cumEval << " " << qTableCoverage << " " << avgState << "\n";
        m_statsStream.flush();

        m_robotPoseStream << stepCount;
        m_robotStateStream << stepCount;
        for (auto& robot : world->getEntities("robot")) {
            Vec2& pos = robot.getComponent<CTransform>().p;
            CSteer& steer = robot.getComponent<CSteer>();
            std::shared_ptr<TailController> ctrl = std::dynamic_pointer_cast<TailController>(robot.getComponent<CController>().controller);
            m_robotPoseStream << " " << (int)pos.x << " " << (int)pos.y << " " << ((int)(1000 * steer.angle)) / 1000.0; // Rounding angle to 3 decimals
            m_robotStateStream << " " << ctrl->getStateAsInt();
        }
        m_robotPoseStream << "\n";
        m_robotStateStream << "\n";
        m_robotPoseStream.flush();
        m_robotStateStream.flush();

        for (int puckType=0; puckType<m_config.numPuckTypes; ++puckType) {
            auto puckStream = m_puckPositionStreamVector[puckType];
            *puckStream << stepCount;
            for (auto& puck : world->getEntities("puck")) {
                if (puck.getComponent<CPuckType>().type != puckType) { continue; }
                Vec2& pos = puck.getComponent<CTransform>().p;
                *puckStream << " " << (int)pos.x << " " << (int)pos.y;
            }
            *puckStream << "\n";
            puckStream->flush();
        }
    }

};
