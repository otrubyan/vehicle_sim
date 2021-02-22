#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#define MAX_CROSSWALKS 16
#define MAX_CROSSWALK_PEDS 32

// define some constants
#define ACCELERATION 1; // m/s^2
#define DECELERATION 3; // m/s^2

#define CAPACITY 25;
#define CONSUMPTION_RATE 1
#define RECUPERATION_RATE 1

// define state machine's states
#define CROSS_VIGILANT 0
#define STOP_CROSS_AHEAD 1
#define WAIT_FOR_PEDS 2

#define STOPPING 0
#define COASTING 1
#define ACCELERATING 2

#define fromKmToM(V) (V*1000/3600)

// speed constants
#define CAPTAINSLOW fromKmToM(40)
#define NORMAL fromKmToM(60)
#define FAST fromKmToM(80)
#define BULLITT fromKmToM(100) 
#define MAXDAMAGE fromKmToM(140)

struct Crosswalk
{
    double distanceFromStart; // m
    uint32_t pedCount;
};

typedef struct Crosswalk Crosswalk;

struct RoutePlanner
{
    char* name;
    double speedLimit; // m/s
    double length; // m
    Crosswalk crosswalks[MAX_CROSSWALKS];
    uint8_t crosswalkCount;
};
typedef struct RoutePlanner RoutePlanner;

struct StatusRegister
{
    uint8_t driveSystemState;
    uint8_t pedSafetySystemState;
    double currentSpeed; // m/s
    uint32_t currentEnergy; // units
    double distanceTravelled; // m
    double tripTime; // s
};
typedef struct StatusRegister StatusRegister;

struct Constraints
{
    double acceleration; // m/s^2
    double deceleration; // m/s^2
    uint32_t energyCapacity; // units
    double energyConsumptionRate; // units/s
    double energyRecuperationRate; // units/s  
};
typedef struct Constraints Constraints;

struct Bus
{
    RoutePlanner route;
    StatusRegister status;
    Constraints constraints;
};
typedef struct Bus Bus;


void logMsg(char* msg)
{
    printf("%s\n", msg);
}

int runSpeedSafetySystem(Bus* bus, double timeResolution)
{ 
    return false;
}

int runPedestrianSafetySystem(Bus* bus, double timeResolution)
{
    switch(bus->status.pedSafetySystemState)
    {
        case CROSS_VIGILANT:{
                            
            /*for(int i=0; i!=bus->route.crosswalkCount; i++)
            {

                // a*t^2
                S = bus->status.currentSpeed * time - bus->constraints.deceleration * time^2

                bus->status.currentSpeed;
                if(bus->route.crosswalks[i].distanceFromStart - bus->status.distanceTravelled > 0);
                //pedCount;    
            }
            
            coordinate
        
            if(bus->status.currentSpeed)
            */
            //bus->status.driveSystemState = ACCELERATING;
            
            // if condition is just right - stop the bus            
            bus->status.pedSafetySystemState = STOP_CROSS_AHEAD;

            
            logMsg("Bus is in CROSSING VIGILANT mode.");
            break;
        }
        case STOP_CROSS_AHEAD:{
            logMsg("Bus is STOPPING BEFORE THE CROSSING.");
            
            bus->status.driveSystemState = STOPPING;
            
            if(bus->status.currentSpeed = 0)
                bus->status.pedSafetySystemState = WAIT_FOR_PEDS;

            break;
        }
        case WAIT_FOR_PEDS:{
            //if(bus->route.crosswalk[i].pedCount > 0)
            //else{bus->status.pedSafetySystemState = CROSS_VIGILANT}
            //bus->status.driveSystemState = STOPPING;
            logMsg("Bus is WAITING FOR PEDS to cross.");
            // honk
            break;
        }    
        default:{
            logMsg("No such state, abort program."); 
            return true;
        }
    }
    
    return false;
}

int runDriveSystem(Bus* bus, double timeResolution)
{
    double distanceDelta = 0;

    switch(bus->status.driveSystemState)
    {
        case STOPPING:{
            logMsg("Bus is STOPPING");
            
            double energyRestored = bus->constraints.energyRecuperationRate * timeResolution;
            bus->status.currentEnergy += energyRestored;
            
            if(bus->status.currentEnergy >= bus->constraints.energyCapacity)
                bus->status.currentEnergy = bus->constraints.energyCapacity;
            
            double speedDelta = bus->constraints.deceleration * timeResolution;
            bus->status.currentSpeed -= speedDelta;
            
            distanceDelta = bus->status.currentSpeed * timeResolution - speedDelta * timeResolution;
            
            if(bus->status.currentSpeed <= 0)
                bus->status.currentSpeed = 0;
        
            printf("energyRestored: %f speedDelta: %f distanceDelta: %f\n", energyRestored, speedDelta, distanceDelta);
        
            break;
        }
        case COASTING:{
            logMsg("Bus is COASTING");
            double energyRestored = bus->constraints.energyRecuperationRate * timeResolution;
            bus->status.currentEnergy += energyRestored;
            
            distanceDelta = bus->status.currentSpeed * timeResolution;
            
            if(bus->status.currentEnergy >= bus->constraints.energyCapacity)
                bus->status.currentEnergy = bus->constraints.energyCapacity;

            printf("energyRestored: %f distanceDelta: %f\n", energyRestored, distanceDelta);
            
            break;
        }
        case ACCELERATING:{
            logMsg("Bus is ACCELERATING");

            if(bus->status.currentEnergy == 0)
            {
                bus->status.driveSystemState = COASTING;
                break;
            }

            double energyBurned = bus->constraints.energyConsumptionRate * timeResolution;
            bus->status.currentEnergy = bus->status.currentEnergy - energyBurned;

            double speedDelta = bus->constraints.acceleration * timeResolution;
            bus->status.currentSpeed = bus->status.currentSpeed + speedDelta;

            distanceDelta = bus->status.currentSpeed * timeResolution + speedDelta * timeResolution;

            // speed limiting logic
            if(bus->status.currentSpeed < bus->route.speedLimit)
                bus->status.driveSystemState = ACCELERATING;
            else if(bus->status.currentSpeed > bus->route.speedLimit)
                bus->status.driveSystemState = STOPPING;
            else
                bus->status.driveSystemState = COASTING;              

            printf("energyBurned: %f speedDelta: %f distanceDelta: %f\n", energyBurned, speedDelta, distanceDelta);
          
            break;
        }
        default:{
            logMsg("No such state, abort program."); 
            return true;
        }
    }

    bus->status.distanceTravelled += distanceDelta;
    
    return false;
}

int runBusSimulation(RoutePlanner route)
{
    printf("%s %s\n", "Simulation difficulty:", route.name);
    logMsg("Started.");
    
    // some simulation helper constants
    bool isFinished = false;
    bool isInterrupted = false;

    double timeResolution = 1; // seconds per simulation tick
    
    // prepare bus to ride
    Bus bus;    
    Constraints constraints;
    constraints.acceleration = ACCELERATION;
    constraints.deceleration = DECELERATION;
    constraints.energyCapacity = CAPACITY;
    constraints.energyConsumptionRate = CONSUMPTION_RATE;
    constraints.energyRecuperationRate = RECUPERATION_RATE;  
    bus.constraints = constraints;
    
    StatusRegister initStatus;
    initStatus.driveSystemState = ACCELERATING;
    initStatus.pedSafetySystemState = CROSS_VIGILANT;
    initStatus.currentSpeed = 0;
    initStatus.distanceTravelled = 0;
    initStatus.currentEnergy = bus.constraints.energyCapacity;
    initStatus.tripTime = 0;
    bus.status = initStatus;
    
    bus.route = route;
      
    // run the machine
    for(uint32_t simCnt = 0; !isFinished && !isInterrupted; simCnt++)
    {
        // wait timer for human readable experience
        usleep(100000);
        
        // pedestrian safety system
        isInterrupted = runPedestrianSafetySystem(&bus, timeResolution);
        
        // drive system
        isInterrupted = runDriveSystem(&bus, timeResolution);
        
        // propagate bus     
        bus.status.tripTime = simCnt*timeResolution;
 
        // check if trip is over        
        if(bus.status.distanceTravelled >= route.length)
            isFinished = true;
    
        printf("spd lmt: %f, spd %f, nrg %d, dist %f, time %f\n", bus.route.speedLimit, bus.status.currentSpeed, bus.status.currentEnergy, bus.status.distanceTravelled, bus.status.tripTime);
    }

    logMsg("Finished.");
}

uint32_t peds()
{
    return rand()%MAX_CROSSWALK_PEDS;
}

int main()
{
    double routeLength = 1000;

    // prepare test routes
    RoutePlanner easy = {"Hey, not too rough", CAPTAINSLOW, routeLength, {350.0, peds()}, 1};
    //RoutePlanner medium = {"Hurt me plenty", NORMAL, routeLength, {{350, peds()}, {700, peds()}}, 2};
    //RoutePlanner hard = {"Ultra-Violence", FAST, routeLength, {{200, peds()}, {400, peds()}, {600, peds()}, {800, peds()}}, 4};
    //RoutePlanner itsgonnahurt = {"Nightmare!", BULLITT, routeLength, {{100, peds()}, {200, peds()}, {300, peds()}, {400, peds()}, {500, peds()}, {600, peds()}, {700, peds()}, {800, peds()}, {900, peds()}}, 9};

    // special test routes
    //RoutePlanner easyrider = {"Hey, not too rough", MAXDAMAGE, routeLength, {}, 0};
    
    // run simulations
    runBusSimulation(easy);    
    //runBusSimulation(medium);
    //runBusSimulation(hard);
    //runBusSimulation(itsgonnahurt);
    //runBusSimulation(easyrider);

    return 0;
}

