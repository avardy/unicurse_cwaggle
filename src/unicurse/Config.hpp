#pragma once

#include <string>
#include <fstream>
#include <iostream>

#include "Hash.hpp"

struct Config
{
    size_t gui          = 1;
    size_t numRobots    = 20;
    size_t fakeRobots    = 0;
    double robotRadius  = 10.0;

    double plowLength   = 60.0;
    double plowAngleDeg   = 0.0;
    double finHalfWidth   = 5.0;

    size_t numPucks     = 0;
    size_t numPuckTypes     = 1;
    double puckRadius   = 10.0;

    std::string arenaConfig = "";
    std::string scalarFieldFilename = "";
    std::string redTravelTimeFilename = "";
    std::string greenTravelTimeFilename = "";

    size_t controllerSkip = 0;

    // Simulation Parameters
    double simTimeStep  = 1.0;
    double renderSteps  = 1;
    size_t maxTimeSteps = 0;

    size_t writeDataSkip    = 0;
    std::string dataFilenameBase   = "";
    size_t numTrials = 10;
    std::string evalName = "";

    size_t captureScreenshots          = 0;
    std::string screenshotFilenameBase   = "";

    double maxForwardSpeed = 2;
    double maxAngularSpeed = 0.05; // 2.0; // 0.5

    double angleThreshold = 0;
    double puckSensorFOV = 360;
    double puckSensorDeltaAngle = 10;
    size_t pucksAsRobots = 0;

    double minCurveIntensity = 51;
    double minTurnSteps = 40;

    size_t searchSpinDistMin = 10;
    size_t searchSpinDistMax = 100;
    size_t searchStraightDistMin = 10;
    size_t searchStraightDistMax = 100;

    std::string sensorConfig = "";

    size_t arenaSweep         = 0;
    size_t paramSweep         = 0;

    // Q-Learning Parameters
    size_t manual    = 0;
    size_t updateQTable    = 0;
    size_t episodeTimeSteps = 0;
    size_t numStates    = 0;
    size_t numActions   = 0;
    size_t batchSize    = 1;
    double initialQ     = 0.0;
    double alpha        = 0.1;
    double gamma        = 0.9;
    double epsilon      = 0.1;
    double penaltyDotRadius    = 0;

    HashFunction hashFunction;

    size_t saveQSkip = 0;
    size_t saveQSkipPartial = 0;
    std::string saveQFile;
    std::string saveQFilePartial;
    size_t loadQ = 0;
    size_t loadQPartial = 0;
    std::string loadQFile;
    std::string loadQFilePartial;
    double resetEval    = 0;

    std::vector<double> actions = { };


    Config() {}

    Config(const std::string & filename) {
        load(filename);
    }

    void load(const std::string & filename)
    {
        std::ifstream fin(filename);
        if (fin.fail())
            std::cerr << "Problem opening file: " << filename << std::endl;
        std::string token;
        double tempVal = 0;
        actions = {};
 
        while (fin.good())
        {
            fin >> token;
            if (token == "numRobots")      { fin >> numRobots; }
            else if (token == "fakeRobots")    { fin >> fakeRobots; }
            else if (token == "robotRadius")    { fin >> robotRadius; }
            else if (token == "plowLength")    { fin >> plowLength; }
            else if (token == "plowAngleDeg")    { fin >> plowAngleDeg; }
            else if (token == "finHalfWidth")    { fin >> finHalfWidth; }
            else if (token == "gui")            { fin >> gui; }
            else if (token == "numPucks")       { fin >> numPucks; }
            else if (token == "numPuckTypes")       { fin >> numPuckTypes; }
            else if (token == "puckRadius")     { fin >> puckRadius; }
            else if (token == "arenaConfig")       { fin >> arenaConfig; }
            else if (token == "scalarFieldFilename")       { fin >> scalarFieldFilename; }
            else if (token == "redTravelTimeFilename")       { fin >> redTravelTimeFilename; }
            else if (token == "greenTravelTimeFilename")       { fin >> greenTravelTimeFilename; }
            else if (token == "controllerSkip")     { fin >> controllerSkip; }
            else if (token == "simTimeStep")    { fin >> simTimeStep; }
            else if (token == "renderSteps")     { fin >> renderSteps; }
            else if (token == "maxTimeSteps")   { fin >> maxTimeSteps; }
            else if (token == "writeDataSkip")  { fin >> writeDataSkip; }
            else if (token == "dataFilenameBase")   { fin >> dataFilenameBase; }
            else if (token == "numTrials")   { fin >> numTrials; }
            else if (token == "evalName")   { fin >> evalName; }
            else if (token == "captureScreenshots")   { fin >> captureScreenshots; }
            else if (token == "screenshotFilenameBase")   { fin >> screenshotFilenameBase; }
            else if (token == "maxForwardSpeed")             { fin >> maxForwardSpeed; }
            else if (token == "maxAngularSpeed")        { fin >> maxAngularSpeed; }
            else if (token == "angleThreshold")        { fin >> angleThreshold; }
            else if (token == "puckSensorFOV")        { fin >> puckSensorFOV; }
            else if (token == "puckSensorDeltaAngle")        { fin >> puckSensorDeltaAngle; }
            else if (token == "pucksAsRobots")        { fin >> pucksAsRobots; }
            else if (token == "minCurveIntensity")   { fin >> minCurveIntensity; }
            else if (token == "minTurnSteps")    { fin >> minTurnSteps; }
            else if (token == "searchSpinDistMin")   { fin >> searchSpinDistMin; }
            else if (token == "searchSpinDistMax")    { fin >> searchSpinDistMax; }
            else if (token == "searchStraightDistMin")   { fin >> searchStraightDistMin; }
            else if (token == "searchStraightDistMax")    { fin >> searchStraightDistMax; }
            else if (token == "sensorConfig")   { fin >> sensorConfig; }
            else if (token == "arenaSweep")     { fin >> arenaSweep; }
            else if (token == "paramSweep")     { fin >> paramSweep; }
            else if (token == "manual")      { fin >> manual; }
            else if (token == "updateQTable")      { fin >> updateQTable; }
            else if (token == "episodeTimeSteps")   { fin >> episodeTimeSteps; }
            else if (token == "batchSize")      { fin >> batchSize; }
            else if (token == "maxTimeSteps")   { fin >> maxTimeSteps; }
            else if (token == "initialQ")       { fin >> initialQ; }
            else if (token == "alpha")          { fin >> alpha; }
            else if (token == "gamma")          { fin >> gamma; }
            else if (token == "epsilon")        { fin >> epsilon; }
            else if (token == "penaltyDotRadius")        { fin >> penaltyDotRadius; }
            else if (token == "resetEval")      { fin >> resetEval; }
            else if (token == "saveQFile")     { fin >> saveQSkip >> saveQFile; }
            else if (token == "saveQFilePartial")     { fin >> saveQSkipPartial >> saveQFilePartial; }
            else if (token == "loadQFile")     { fin >> loadQ >> loadQFile; }
            else if (token == "loadQFilePartial")     { fin >> loadQPartial >> loadQFilePartial; }
            else if (token == "hashFunction")   
            { 
                fin >> token;
                hashFunction = Hash::GetHashData(token).Function; 
                numStates    = Hash::GetHashData(token).MaxHashSize;
            }
            else if (token == "actions") 
            { 
                fin >> numActions; 
                for (size_t a = 0; a < numActions; ++a)
                {
                    fin >> tempVal;
                    actions.push_back(tempVal);
                }
            }
        }
    }
};
