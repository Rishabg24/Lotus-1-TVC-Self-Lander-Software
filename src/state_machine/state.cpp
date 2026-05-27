#include "state.h"
#include <Arduino.h>
#include "control/tvc.h"
static FlightState currentState = GROUND_IDLE;
static unsigned long stateEntryTime = 0;
void setFlightState(FlightState newState)
{
  currentState = newState;
  stateEntryTime = millis();
}

void updateFlightState(float accelZ, float altitude, float prevAltitude, unsigned long dt)
{
  switch (currentState)
  {
  case GROUND_IDLE:
    if (accelZ > 3.0) // 
      setFlightState(POWERED_FLIGHT);
    break;

  case POWERED_FLIGHT:
    if (accelZ < 2.0)
      setFlightState(UNPOWERED_FLIGHT);
    break;

  case UNPOWERED_FLIGHT:
    if (altitude < prevAltitude)
    {
      setFlightState(BALLISTIC_DESCENT);
    }
    break;

  case POWERED_DESCENT:

    if (altitude < 1.0)
      setFlightState(LANDING);
    break;

  case LANDING:
    // Remain idle
    break;
  }
}

FlightState getCurrentState() { return currentState; }