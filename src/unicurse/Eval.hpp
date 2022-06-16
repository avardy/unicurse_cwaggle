#pragma once

#include "World.hpp"
#include "Entity.hpp"
#include "Components.hpp"

namespace Eval
{
    double lineGridOccGridOverlapReward(ValueGrid &lineGrid, ValueGrid &occGrid) {
        size_t width = lineGrid.width();
        size_t height = lineGrid.height();

        double total = 0;
        for (size_t x = 0; x < width; ++x) {
            for (size_t y = 0; y < height; ++y) {
                if (lineGrid.get(x, y) > 0 && occGrid.get(x, y) > 0)
                    ++total;
            }
        }
        return total;// / (width * height);
    }

    double sensorOnLineReward(std::shared_ptr<World> world) {

        double n = 0;
        double nOnLine = 0;

        for (auto robot : world->getEntities("robot")) {
            // We need the robot's sensor readings because the line following behaviour should
            // maintain the forward sensor's reading on the line.
            SensorReading reading;
            SensorTools::ReadSensorArray(robot, world, reading);

            if (reading.floorCentre[0] > 0)
                nOnLine++;
            //else
            //    nOnLine--;
            n++;
        }
        
        return nOnLine / n;
    }

    double PuckCenterSSD(std::shared_ptr<World> world)
    {
        Vec2 averagePosition;

        for (auto e : world->getEntities("puck"))
        {
            auto & t = e.getComponent<CTransform>();

            averagePosition += t.p;
        }

        averagePosition /= world->getEntities("puck").size();

        double ssd = 0;

        for (auto e : world->getEntities("puck"))
        {
            ssd += e.getComponent<CTransform>().p.dist(averagePosition);
        }

        return 800 - (ssd / world->getEntities("puck").size());
    }

    double PuckAvgThresholdDiff(std::shared_ptr<World> world, double t1, double t2)
    {
        double sum = 0;
        auto & grid = world->getGrid(0);
        for (auto e : world->getEntities("puck"))
        {
            auto & t = e.getComponent<CTransform>();
            size_t gridX = (size_t)(grid.width() * (t.p.x / world->width()));
            size_t gridY = (size_t)(grid.height() * (t.p.y / world->height()));
            double gridVal = grid.get(gridX, gridY);

            double diff = 0;
            if (gridVal < t1) { diff = abs(gridVal - t1); } 
            else if (gridVal > t2) { diff = abs(gridVal - t2); }
            sum += diff;
        }

        double maxDiff = std::max(t1, 1-t2);
        return 1 - ((sum / world->getEntities("puck").size()) / maxDiff);
    }

    // For debuggging placement of pucks on top of other entities.
    double PucksOnTopOfAnything(std::shared_ptr<World> world, std::string puckType)
    {
        double total = 0;
        for (auto e1 : world->getEntities(puckType))
        {
            assert(e1.hasComponent<CTransform>());
            Vec2 pos1 = e1.getComponent<CTransform>().p;

            for (auto e2 : world->getEntities())
            {
                if (e1.id() != e2.id() && e2.hasComponent<CTransform>()) {
                    Vec2 pos2 = e2.getComponent<CTransform>().p;
                    if (fabs(pos1.x - pos2.x) < 0.001 && fabs(pos1.y - pos2.y) < 0.001)
                        total++;
                }
            }
        }
        
        return total;
    }

    // For debuggging placement of pucks on top of other entities.
    double NanPucks(std::shared_ptr<World> world, std::string puckType)
    {
        double total = 0;
        for (auto e1 : world->getEntities(puckType))
        {
            assert(e1.hasComponent<CTransform>());
            Vec2 pos = e1.getComponent<CTransform>().p;

            if (isnan(pos.x) || isnan(pos.y))
                total++;
        }
        
        return total;
    }

    // For debugging behaviour at the border
    double PucksCloseToBorder(std::shared_ptr<World> world, std::string puckType, double thresholdDistance)
    {
        double n = 0;
        double total = 0;
        for (auto e : world->getEntities(puckType))
        {
            assert(e.hasComponent<CTransform>());

            Vec2 pos = e.getComponent<CTransform>().p;

            if (pos.x < thresholdDistance ||
                pos.x >= world->width() - thresholdDistance ||
                pos.y < thresholdDistance ||
                pos.y >= world->height() - thresholdDistance)
            {
                total++;
            }
            n++;
        }
        
        if (isnan(total) || n == 0) { return 1; }
        return total / n;
    }

    double PuckGridValues(std::shared_ptr<World> world, size_t puckType, int gridIndex)
    {
        auto & grid = world->getGrid(gridIndex);
        if (grid.width() == 0) { return 0; }

        double total = 0;
        double n = 0;
        for (auto e : world->getEntities("puck"))
        {
            assert(e.hasComponent<CTransform>());
            if (e.getComponent<CPuckType>().type != puckType) { continue; }

            Vec2 pos = e.getComponent<CTransform>().p;

            size_t gX = (size_t)round(grid.width()  * pos.x / world->width());
            size_t gY = (size_t)round(grid.height() * pos.y / world->height());
            total += grid.get(gX, gY);
            n++;
        }
        
        // Any problems?  Return 0.
        if (isnan(total) || n == 0) { return 0; }

        return total / n;
    }

    /**
     * Computed the biggest difference between any two grid values that are 
     * within the given radius of each other.
     */
    double BiggestGridValueDifference(std::shared_ptr<World> world, int gridIndex, int radius)
    {
        auto & grid = world->getGrid(gridIndex);

        double biggestDiff = 0;
        int w = grid.width();
        int h = grid.height();
        for (int i=radius; i < w - radius; ++i) {
            for (int j=radius; j < h - radius; ++j) {

                // Generally now assume grid and world dimensions are
                // the same, so this is not really necessary.  But will
                // this assumption always hold?
                //size_t gX = (size_t)round(w * i / world->width());
                //size_t gY = (size_t)round(h * j / world->height());
                //double centralValue = grid.get(gX, gY);
                double centralValue = grid.get(i, j);

                for (int di=-radius; di <= radius; ++di) {
                    for (int dj=-radius; dj <= radius; ++dj) {
                        if (sqrt(di*di + dj*dj) <= radius) {
                            //gX = (size_t)round(w * (i + di) / world->width());
                            //gY = (size_t)round(h * (j + dj) / world->height());
                            //double diff = fabs(centralValue - grid.get(gX, gY));
                            double diff = fabs(centralValue - grid.get(i + di, j + dj));
                            if (diff > biggestDiff)
                                biggestDiff = diff;
                        }
                    }
                }

            }
        }

        return biggestDiff;
    }

    double PuckSSDFromIdealPosition(std::shared_ptr<World> world, size_t puckType, Vec2 idealPosition)
    {
        double ssd = 0;

        double n = 0;
        for (auto e : world->getEntities("puck"))
        {
            if (e.getComponent<CPuckType>().type != puckType) { continue; }

            ssd += e.getComponent<CTransform>().p.distSq(idealPosition);
            n++;
//auto t = e.getComponent<CTransform>();
//std::cout << t.p.x << "," << t.p.y << std::endl;
        }
//        assert(!isnan(ssd));

        return ssd / (n * pow(hypot(world->width(), world->height()), 2));
    }

    // For debugging behaviour at the border
    double ProportionSlowedRobots(std::shared_ptr<World> world)
    {
        double n = 0;
        double total = 0;
        for (auto e : world->getEntities("robot")) {
            total++;
            CSteer steer = e.getComponent<CSteer>();

            if (steer.slowedCount > 0)
                n++;
        }
        
        return n / total;
    }
}
