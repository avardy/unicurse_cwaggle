#pragma once

//#include "Components.hpp"
#include "EntityControllers.hpp"
#include "Simulator.hpp"
#include "ValueGrid.hpp"
#include "World.hpp"
#include "WorldUtils.hpp"
#include "Intersect.h"

#include "Config.hpp"

#include <random>
#include <sstream>

using namespace std;

namespace unicurse_world {

/**
 * If the given position is within the given radius of any lines, return false.  Otherwise
 * we assume the position is okay to occupy and return true.
 */
bool checkPosition(std::shared_ptr<World> world, Vec2 p, double radius)
{
    for (auto lineEntity : world->getEntities("line")) {
        auto & line = lineEntity.getComponent<CLineBody>();
        if (Intersect::checkCircleSegmentIntersection(line.s, line.e, p, line.r + radius))
            return false;
    }
    return true;
}

Entity addRobot(std::shared_ptr<World> world, Config config)
{
    Entity robot = world->addEntity("robot");

    // This position will later be overwritten.
    Vec2 rPos(0, 0);
    robot.addComponent<CTransform>(rPos);
    if (!config.fakeRobots) {
        robot.addComponent<CCircleBody>(config.robotRadius, true);
        robot.addComponent<CCircleShape>(config.robotRadius);
        robot.addComponent<CColor>(50, 50, 100, 200);
    }

    double plowAngleRad = config.plowAngleDeg * M_PI / 180.0;

    // Wedge shaped plow at the front of the robot.
    if (config.plowLength > 0 && !config.fakeRobots)
        robot.addComponent<CPlowBody>(config.plowLength, config.robotRadius / 10, config.plowAngleDeg);
        //robot.addComponent<CPlowBody>(config.plowLength, config.robotRadius, config.plowAngleDeg);

    // Add a fin to capture pucks.
    if (config.finHalfWidth > 0) {
        double finX = -config.plowLength;
        double finY = config.finHalfWidth;
        robot.addComponent<CAttachedLineBody>(Vec2(finX, finY), Vec2(finX, -finY), 1);
    }

    auto& sensors = robot.addComponent<CSensorArray>();

    /*
    if (config.sensorConfig == "arc_3") {
        // An arc of grid sensors, with 1 at angle 0 and an additional nSensorsPerSide on the right and left.
        // (total number of sensors is 1 + 2*nSensorsPerSide).
        double radius = 0.35 * config.robotRadius;
        int nSensorsPerSide = 1;
        double fovPerSide = 45;
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, 0, radius));
        for (int i=1; i<=nSensorsPerSide; ++i) {
            double angle = fovPerSide * i / (double) nSensorsPerSide;
            sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, angle, radius));
            sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, -angle, radius));
        }
    } else if (config.sensorConfig == "nearline_3") {
        // Three grid sensors in a line, near the centre of the bot.
        double forwardDistance = 0.35 * config.robotRadius;
        double lateralDistance = 0.25 * config.robotRadius;
        double angle = atan(lateralDistance/forwardDistance) * 180.0 / M_PI;
        double distance = hypot(lateralDistance, forwardDistance);
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, 0, forwardDistance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, angle, distance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, -angle, distance));
    } else 
    */
    if (config.sensorConfig == "nearline_3_rgb") {
        // Three grid sensors in a line, near the centre of the bot.
        double forwardDistance = 0.35 * config.robotRadius;
        double lateralDistance = 0.25 * config.robotRadius;
        double angle = atan(lateralDistance/forwardDistance) * 180.0 / M_PI;
        double distance = hypot(lateralDistance, forwardDistance);

        for (int i=0; i<3; ++i) {
            sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "centre", i, 0, forwardDistance));
            sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "right", i, angle, distance));
            sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "left", i, -angle, distance));
        }
    } /* else if (config.sensorConfig == "nearline_5") {
        // Five grid sensors in a line, near the centre of the bot.
        double forwardDistance = 0.35 * config.robotRadius;
        double lateralDistance = 0.25 * config.robotRadius;
        double angle = atan(lateralDistance/forwardDistance) * 180.0 / M_PI;
        double distance = hypot(lateralDistance, forwardDistance);
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, 0, forwardDistance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, angle, distance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, -angle, distance));
        lateralDistance += 0.25 * config.robotRadius;
        angle = atan(lateralDistance/forwardDistance) * 180.0 / M_PI;
        distance = hypot(lateralDistance, forwardDistance);
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, angle, distance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, -angle, distance));
    } else if (config.sensorConfig == "farline_3") {
        // Three grid sensors in a line, out to the front of the bot.
        double forwardDistance = 0.95 * config.robotRadius;
        double lateralDistance = 0.25 * config.robotRadius;
        double angle = atan(lateralDistance/forwardDistance) * 180.0 / M_PI;
        double distance = hypot(lateralDistance, forwardDistance);
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, 0, forwardDistance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, angle, distance));
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "gridVec", 0, -angle, distance));
    }
    */
    else {
        cerr << "NO SUCH VALUE FOR config.sensorConfig!" << endl;
    }

    for (int i=0; i<3; ++i)
        sensors.gridSensors.push_back(make_shared<GridSensor>(robot, "edge", i, 90, 16));
    
    double r = config.robotRadius;

    // Using a ring of PuckSensors
    for (int t=0; t<config.numPuckTypes; ++t) {

        std::vector<shared_ptr<PuckSensor>> puckSensors;

        // The radius of each puck sensor is set to be just a little less (that's
        // the factor in front) than the plow's reach.
        double radius = 0.65 * (config.plowLength - config.robotRadius) / 2.0;
        double distance = radius + config.robotRadius;

        /*
        int n = 8;
        double halfFOV = 0.5 * config.puckFOV;
        if (halfFOV == 180)
            halfFOV -= halfFOV / n;  // prevents overlapping sensors if halfFOV = 180

        for (int i=0; i<n; ++i) {
            double angle = -halfFOV + (i / (double)(n - 1)) * 2 * halfFOV;
            puckSensors.push_back(std::make_shared<PuckSensor>(robot, "pucks", "puck", t, angle, distance, radius));
        }
        */
        double halfFOV = 0.5 * config.puckSensorFOV;
        double deltaAngle = config.puckSensorDeltaAngle;
        if (halfFOV == 180)
            halfFOV -= 0.5*deltaAngle;  // prevents overlapping sensors if halfFOV = 180
        // Insert those sensors with negative angle.
        for (double angle = -halfFOV; angle <= 0; angle += deltaAngle)
            puckSensors.push_back(std::make_shared<PuckSensor>(robot, "pucks", "puck", t, angle, distance, radius));
        // Insert sensors with positive angle, note that we are inserting first at the end of
        // the vector, but maintaining the same insertion point so that even though we're
        // inserting in CCW order, the sensor indices will still be in CW order.
        int s = puckSensors.size();
        for (double angle = halfFOV; angle > 0; angle -= deltaAngle) {
            auto mid = puckSensors.begin() + s;
            puckSensors.insert(mid, std::make_shared<PuckSensor>(robot, "pucks", "puck", t, angle, distance, radius));
        }

        sensors.puckSensorsByType.push_back( puckSensors );
    }
    //sensors.puckSensors.push_back(std::make_shared<PuckSensor>(robot, "leftPucks", "puck", -40, 1.25*r, 0.75*r));
    //sensors.puckSensors.push_back(std::make_shared<PuckSensor>(robot, "rightPucks", "puck", 40, 1.25*r, 0.75*r));

    /*
    // The puck sensor is defined by two circles, the first is forward of the robot and represents the
    // sensor's limited field of view.  The second is centred on the robot and represents the sensor's
    // limited range.
    std::vector<SensingCircle> circles;
    circles.push_back(SensingCircle(0, config.fovCircleDistance, config.fovCircleRadius, true));
    circles.push_back(SensingCircle(0, 0, config.rangeCircleRadius, true));

    auto fancyLeftPuckSensor = std::make_shared<FancySensor>(robot, "puck", "left", circles, true, false);
    auto fancyRightPuckSensor = std::make_shared<FancySensor>(robot, "puck", "right", circles, false, true);
    sensors.fancySensors.push_back(fancyLeftPuckSensor);
    sensors.fancySensors.push_back(fancyRightPuckSensor);
    */

    double robotSensorDistance = 1.0 * r;
    double robotSensorRadius = 0.9 * r;
    sensors.robotSensors.push_back(make_shared<RobotSensor>(robot, "robotAhead", 0, robotSensorDistance, robotSensorRadius));

    robot.addComponent<CControllerVis>();

    return robot;
}

void addRobots(shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, 
               uniform_real_distribution<double> angleRNG, default_random_engine &rng, Config config)
{
    int numRobots = (int)config.numRobots;
    for (size_t r = 0; r < numRobots; r++) {
        Entity robot = addRobot(world, config);
        auto& transform = robot.getComponent<CTransform>();

        do {
            transform.p.x = config.robotRadius + Xrng(rng);
            transform.p.y = config.robotRadius + Yrng(rng);
        } while (!checkPosition(world, transform.p, config.robotRadius));

        robot.addComponent<CSteer>();
        auto & steer = robot.getComponent<CSteer>();
        steer.angle = angleRNG(rng);
    }
}


Entity addPuck(size_t type, size_t red, size_t green, size_t blue, shared_ptr<World> world, size_t puckRadius)
{
    Entity puck = world->addEntity("puck");

    // This position will later be overwritten.
    Vec2 pPos(0, 0);
    puck.addComponent<CTransform>(pPos);
    puck.addComponent<CCircleBody>(puckRadius);
    puck.addComponent<CCircleShape>(puckRadius);
    puck.addComponent<CColor>(red, green, blue, 255);
    puck.addComponent<CPuckType>(type);

    return puck;
}

void addPucks(size_t type, size_t red, size_t green, size_t blue, shared_ptr<World> world, uniform_int_distribution<int> Xrng, uniform_int_distribution<int> Yrng, default_random_engine &rng,
    Config config)
{
    int puckRadius = (int)config.puckRadius;
    for (size_t r = 0; r < config.numPucks; r++) {
        Entity puck = addPuck(type, red, green, blue, world, puckRadius);

        auto& transform = puck.getComponent<CTransform>();
        do {
            transform.p.x = config.puckRadius + Xrng(rng);
            transform.p.y = config.puckRadius + Yrng(rng);
        } while (!checkPosition(world, transform.p, config.robotRadius));

        //std::cerr << puck.id() << ", x, y: " << transform.p.x << ", " << transform.p.y << std::endl;
    }
}

// A probe is a robot without a body or a controller.
Entity addProbe(std::shared_ptr<World> world, Config config)
{
    Entity probe = world->addEntity("probe");

    // This position will later be overwritten.
    Vec2 rPos(world->width()/2, world->height()/2);
    probe.addComponent<CTransform>(rPos);
    probe.addComponent<CCircleBody>(5, true);
    probe.addComponent<CCircleShape>(5);
    probe.addComponent<CColor>(100, 100, 255, 100);

    return probe;
}

void addFakeRobots(shared_ptr<World> world, Config config)
{
    double w = world->width();
    double h = world->height();
    double gap = 40;
    double nAngles = 16;
    double dFromCentre = 0;

    for (double y = gap/2; y < h; y += gap) {
        for (double x = gap/2; x < w; x += gap) {

            // Position (x, y) is used as a centre, but for each angle we push a little in that
            // direction to arrive at (u, v) the robot centre for that angle.
            for (double angle = 0; angle < 2*M_PI; angle += (2*M_PI) / (nAngles)) {
                double u = x + dFromCentre * cos(angle);
                double v = y + dFromCentre * sin(angle);

                if (checkPosition(world, Vec2{u, v}, 0)) {
                    Entity robot = addRobot(world, config);
                    auto& transform = robot.getComponent<CTransform>();
                    transform.p.x = u;
                    transform.p.y = v;

                    robot.addComponent<CSteer>();
                    auto & steer = robot.getComponent<CSteer>();
                    steer.angle = angle;
                }
            }
        }
    }
}

void randomizeWorld(shared_ptr<World> world, default_random_engine &rng, Config config)
{
    // Prepare to generate random x and y positions.
    int randXDomain = world->width() - 2 * (int) config.robotRadius;
    int randYDomain = world->height() - 2 * (int) config.robotRadius;
    std::uniform_int_distribution<int> robotXrng(0, randXDomain);
    std::uniform_int_distribution<int> robotYrng(0, randYDomain);

    // Prepare to generate robot angles in radians
    uniform_real_distribution<double> robotAngleRNG(-M_PI, M_PI);

    for (auto & robot : world->getEntities("robot")) {
        auto& transform = robot.getComponent<CTransform>();

        do {
            transform.p.x = config.robotRadius + robotXrng(rng);
            transform.p.y = config.robotRadius + robotYrng(rng);
        } while (!checkPosition(world, transform.p, config.robotRadius));

        auto & steer = robot.getComponent<CSteer>();
        steer.angle = robotAngleRNG(rng);
    }

    randXDomain = world->width() - 2*(int)config.puckRadius;
    randYDomain = world->height() - 2*(int)config.puckRadius;
    std::uniform_int_distribution<int> puckXrng(0, randXDomain);
    std::uniform_int_distribution<int> puckYrng(0, randYDomain);

    for (auto & puck : world->getEntities("puck")) { 
        auto& transform = puck.getComponent<CTransform>();
        do {
            transform.p.x = config.puckRadius + puckXrng(rng);
            transform.p.y = config.puckRadius + puckYrng(rng);
        } while (!checkPosition(world, transform.p, config.puckRadius));
    }
}

void addGoal(double x, double y, double radius, size_t red, size_t green, size_t blue, shared_ptr<World> world)
{
    Entity goal = world->addEntity("goal");
    Vec2 pos(x, y);
    goal.addComponent<CTransform>(pos);
    goal.addComponent<CCircleShape>(radius);
    goal.addComponent<CColor>(red, green, blue, 75);
}

shared_ptr<World> GetWorld(default_random_engine &rng, Config config)
{
    ValueGrid valueGrid0(config.scalarFieldFilename, 0, 0.0);
    ValueGrid valueGrid1(config.scalarFieldFilename, 1, 0.0);
    ValueGrid valueGrid2(config.scalarFieldFilename, 2, 0.0);

    size_t width = valueGrid0.width();
    size_t height = valueGrid0.height();

    auto world = std::make_shared<World>(width, height);

    // Add the two-ends to form the stadium shape.  Note that we don't do this for the live configuration
    // because the physical table plays the role of confining both pucks and robots.
    if (config.arenaConfig == "sim_stadium_no_wall" || config.arenaConfig == "sim_stadium_one_wall" || config.arenaConfig == "sim_stadium_one_wall_double"
        || config.arenaConfig == "sim_stadium_two_walls" || config.arenaConfig == "sim_stadium_three_walls") {
        // Create the left and right arcs to match our stadium-shaped air hockey table
        WorldUtils::AddLineBodyArc(world, 64, width/3, height/2, height/2, -3*M_PI/2, -M_PI/2, 100);
        WorldUtils::AddLineBodyArc(world, 64, 2*width/3, height/2, height/2, -M_PI/2, M_PI/2, 100);

        // Create top and bottom line bodies.  These are not really needed for
        // the simulation aspect, but moreso for visualization.
        Entity topWall = world->addEntity("line");
        Entity botWall = world->addEntity("line");
        topWall.addComponent<CLineBody>(Vec2(0, 0), Vec2(width-1, 0), 1);
        botWall.addComponent<CLineBody>(Vec2(0, height), Vec2(width-1, height), 1);
    }

    // Whenever we add a wall here, we actually create two different entites:
    //  line - Used for regular collisions.
    //  visibility_line - Used only to check for visibility from one part of the arena to another
    //
    if (config.arenaConfig == "4k_one_wall_div_4") {
        Entity wall = world->addEntity("line");
        wall.addComponent<CLineBody>(Vec2(4096/8, 0), Vec2(4096/8, 3*2160/(4*4)), 30);

        Entity vWall = world->addEntity("visibility_line");
        vWall.addComponent<CLineBody>(wall.getComponent<CLineBody>());

    }
    if (config.arenaConfig == "live_one_wall") {
        Entity wall = world->addEntity("line");
        wall.addComponent<CLineBody>(Vec2(606, 0), Vec2(606, 404), 25);

        Entity vWall = world->addEntity("visibility_line");
        vWall.addComponent<CLineBody>(wall.getComponent<CLineBody>());

    }
    if (config.arenaConfig == "sim_stadium_one_wall" || config.arenaConfig == "sim_stadium_one_wall_double" ||
        config.arenaConfig == "sim_stadium_two_walls" || config.arenaConfig == "sim_stadium_three_walls" ) {
        Entity wall = world->addEntity("line");
        if (config.arenaConfig == "sim_stadium_one_wall_double")
            // Double-thickness wall
            wall.addComponent<CLineBody>(Vec2(width/2, 0), Vec2(width/2, 0.625*height), 32);
        else
            wall.addComponent<CLineBody>(Vec2(width/2, 0), Vec2(width/2, 0.625*height), 16);

        Entity vWall = world->addEntity("visibility_line");
        vWall.addComponent<CLineBody>(wall.getComponent<CLineBody>());
    }
    if (config.arenaConfig == "sim_stadium_two_walls" || config.arenaConfig == "sim_stadium_three_walls" ) {
        Entity wall = world->addEntity("line");
        wall.addComponent<CLineBody>(Vec2(3*width/4, height), Vec2(3*width/4, 0.4*height), 16);

        Entity vWall = world->addEntity("visibility_line");
        vWall.addComponent<CLineBody>(wall.getComponent<CLineBody>());
    }
    if (config.arenaConfig == "sim_stadium_three_walls" ) {
        Entity wall = world->addEntity("line");
        wall.addComponent<CLineBody>(Vec2(0, 0.6*height), Vec2(width/4, 0.6*height), 16);

        Entity vWall = world->addEntity("visibility_line");
        vWall.addComponent<CLineBody>(wall.getComponent<CLineBody>());
    }

    world->update();

    if (config.fakeRobots)
        addFakeRobots(world, config);

    // Prepare to generate random x and y positions for robots.
    int randXDomain = world->width() - 2 * (int)config.robotRadius;
    int randYDomain = world->height() - 2 * (int)config.robotRadius;
    uniform_int_distribution<int> robotXrng(0, randXDomain);
    uniform_int_distribution<int> robotYrng(0, randYDomain);

    // Prepare to generate robot angles in radians
    uniform_real_distribution<double> robotAngleRNG(-M_PI, M_PI);

    addRobots(world, robotXrng, robotYrng, robotAngleRNG, rng, config);

    // Prepare to generate random x and y positions for pucks.
    // Random number generators for x- and y- position.
    randXDomain = world->width() - 2 * (int)config.puckRadius;
    randYDomain = world->height() - 2 * (int)config.puckRadius;
    uniform_int_distribution<int> puckXrng(0, randXDomain);
    uniform_int_distribution<int> puckYrng(0, randYDomain);

    if (config.numPuckTypes >= 1) {
        addPucks(0, 255, 0, 0, world, puckXrng, puckYrng, rng, config);
        //addGoal(config.redGoalX, config.redGoalY, config.redGoalRadius, 255, 0, 0, world);
    }
    if (config.numPuckTypes >= 2) {
        addPucks(1, 0, 255, 0, world, puckXrng, puckYrng, rng, config);
        //addGoal(config.greenGoalX, config.greenGoalY, config.greenGoalRadius, 0, 255, 0, world);
    }

    //addProbe(world, config);

    world->addGrid(valueGrid0);
    world->addGrid(valueGrid1);
    world->addGrid(valueGrid2);

    // A grid to show the robot's occupancy over time---used to evaluate line-following performance.
    world->addGrid(ValueGrid{width, height, 0, 0});

    if (config.numPuckTypes >= 1) {
        ValueGrid redTravelTime(config.redTravelTimeFilename, 0, 0.0);
        world->addGrid(redTravelTime);
    }
    if (config.numPuckTypes >= 2) {
        ValueGrid greenTravelTime(config.greenTravelTimeFilename, 0, 0.0);
        world->addGrid(greenTravelTime);
    }

    // Four grids (red, green, blue and white) for visualization.
    //world->addGrid(ValueGrid{width, height, 0, 0});
    //world->addGrid(ValueGrid{width, height, 0, 0});
    //world->addGrid(ValueGrid{width, height, 0, 0});
    //world->addGrid(ValueGrid{width, height, 0, 0});

    world->update();
    return world;
}

};