#pragma once

#include <functional>

#include "CWaggle.h"

typedef std::function<size_t(const SensorReading &)> HashFunction;

struct HashFunctionData
{
    HashFunction Function;
    size_t MaxHashSize = 0;
};

namespace Hash
{
    const HashFunctionData & GetHashData(const std::string & hashFunctionName);

    size_t GridArc3(const SensorReading & reading)
    {
        const static std::string & name = "GridArc3";
        const static size_t maxHashSize = GetHashData(name).MaxHashSize;
        size_t hash = 0;

        hash += (reading.floorCentre[0] > 0) ? (1 << 0) : 0;
        hash += (reading.floorRight[0] > 0) ? (1 << 1) : 0;
        hash += (reading.floorLeft[0] > 0) ? (1 << 2) : 0;

        if (hash >= maxHashSize) { std::cerr << "WARNING: " << name << " hash size too large: " << hash; }
        return hash;
    }

    /*
    size_t PuckMid4(const SensorReading & reading)
    {
        const static std::string & name = "PuckMid4";
        const static size_t maxHashSize = GetHashData(name).MaxHashSize;
        size_t hash = 0;

        hash += (reading.gridLeft > 0) ? 0 : (1 << 0);
        hash += (reading.gridRight > 0) ? 0 : (1 << 1);

        size_t midNestReading = (size_t)(floor(reading.gridCentre * 4));
        if (midNestReading == 4) midNestReading = 3;
        hash += midNestReading * (1 << 2);

        if (hash >= maxHashSize) { std::cerr << "WARNING: " << name << " hash size too large: " << hash; }
        return hash;
    }
    */

    /*
    size_t PuckMid16(const SensorReading & reading)
    {
        const static std::string & name = "PuckMid16";
        const static size_t maxHashSize = GetHashData(name).MaxHashSize;
        size_t hash = 0;

        hash += (reading.leftPucks == 0) ? 0 : (1 << 0);
        hash += (reading.rightPucks == 0) ? 0 : (1 << 1);

        size_t midNestReading = (size_t)(floor(reading.gridCentre * 16));
        if (midNestReading == 16) midNestReading = 15;
        hash += midNestReading * (1 << 2);

        if (hash >= maxHashSize) { std::cerr << "WARNING: " << name << " hash size too large: " << hash; }
        return hash;
    }
    */

    const HashFunctionData & GetHashData(const std::string & hashFunctionName)
    {
        static std::map<std::string, HashFunctionData> hashData;

        // set up the hash function data the first time we call this function
        // it will only be called once per experiment
        if (hashData.empty())
        {
            hashData["GridArc3"]    = { GridArc3,       (1 << 3) };
            //hashData["PuckMid4"]    = { PuckMid4,       (1 << 4) };
            //hashData["PuckMid16"]   = { PuckMid16,      (1 << 6) };
        }

        if (hashData.find(hashFunctionName) == hashData.end())
        {
            std::cerr << "Warning: Hash Function Not Found: " << hashFunctionName << "\n";
            exit(-1);
        }

        return hashData[hashFunctionName];
    }

}