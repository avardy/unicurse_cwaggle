#pragma once

#include <sstream>

#include "Entity.hpp"
#include "Sensors.hpp"
#include "World.hpp"

struct SensorReading {

    double floorCentre[3] = {0, 0, 0};
    double floorRight[3] = {0, 0, 0};
    double floorLeft[3] = {0, 0, 0};
    double floorEdge[3] = {0, 0, 0};

    std::vector< std::vector<double> > pucksByType;

    double robotAhead = 0;

    std::string toString()
    {
        std::stringstream ss;

        ss << "floorLeft: (" << floorLeft[0] << ", " << floorLeft[1] << ", " << floorLeft[2] << ")\n";
        ss << "floorCentre: (" << floorCentre[0] << ", " << floorCentre[1] << ", " << floorCentre[2] << ")\n";
        ss << "floorRight: (" << floorRight[0] << ", " << floorRight[1] << ", " << floorRight[2] << ")\n";
        ss << "floorEdge: (" << floorEdge[0] << ", " << floorEdge[1] << ", " << floorEdge[2] << ")\n";

        for (int t=0; t<pucksByType.size(); ++t) {
            ss << "\npuck type " << t << ": ";
            for (int i=0; i<pucksByType[t].size(); ++i) {
                ss << pucksByType[t][i] << ", ";
            }
        }
        ss << "\trobotAhead: " << robotAhead;

        return ss.str();
    }
};

namespace SensorTools {

inline void ReadSensorArray(Entity e, std::shared_ptr<World> world, SensorReading& reading)
{
    reading = {};

    auto& sensors = e.getComponent<CSensorArray>();

    for (auto& sensor : sensors.gridSensors) {
        int channel = sensor->m_gridIndex;

        if (sensor->name() == "centre")
            reading.floorCentre[channel] = sensor->getReading(world);
        else if (sensor->name() == "left")
            reading.floorLeft[channel] = sensor->getReading(world);
        else if (sensor->name() == "right")
            reading.floorRight[channel] = sensor->getReading(world);
        else if (sensor->name() == "edge")
            reading.floorEdge[channel] = sensor->getReading(world);
    }

    for (int t=0; t<sensors.puckSensorsByType.size(); ++t) {
        std::vector<double> pucks;

        for (auto & sensor : sensors.puckSensorsByType[t]) {
            double value = sensor->getReading(world);
            pucks.push_back( value );
        }

        reading.pucksByType.push_back( pucks );
    }

    for (auto& sensor : sensors.robotSensors) {
        if (sensor->name() == "robotAhead")
            reading.robotAhead = sensor->getReading(world);
    }
}

}