enum FlightState{
    GROUND_IDLE,
    POWERED_FLIGHT,
    UNPOWERED_FLIGHT,
    BALLISTIC_DESCENT,
    POWERED_DESCENT,
    LANDING
};

void updateFlightState(float accel_z, float alitutde, float prev_altitude, unsigned long dt);
FlightState getCurrentState();
void setFlightState(FlightState newState);

