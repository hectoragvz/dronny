struct SimClock {
    double dt;
    double current_time;

    SimClock(double dt) : dt(dt), current_time(0.0) {}

    void tick() {
        current_time += dt;
    }
};
