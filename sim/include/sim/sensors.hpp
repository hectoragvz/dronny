#include <random>

// The barometer takes the true altitude and returns it with some random noise added.

class Barometer {
public:
    double noise_std;
    std::mt19937 engine{std::random_device{}()};
    std::normal_distribution<double> dist;

    Barometer(double noise_std) : dist(0.0, noise_std) {}

    double read(double true_altitude) {
        return true_altitude + dist(engine);
    }
};

// The gyroscope takes the true pitch rate and returns it with noise plus a slowly drifting bias.
class Gyroscope {
    public:
        std::mt19937 engine{std::random_device{}()};
        std::normal_distribution<double> noise_dist;
        std::normal_distribution<double> drift_dist;
        double driftBias = 0.0;

        Gyroscope(double noise_std, double drift_std)
            : noise_dist(0.0, noise_std)
            , drift_dist(0.0, drift_std) {}

        double read(double true_pitch_rate) {
            driftBias += drift_dist(engine);
            return true_pitch_rate + driftBias + noise_dist(engine);
        }
};
