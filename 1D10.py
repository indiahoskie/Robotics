#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include "utils.hpp"   // declares findMinDist, crossProduct
#include "robot.hpp"   // your robot interface; adjust include as needed

// ---- Tunables (start here) ----
static constexpr double SETPOINT_M      = 0.50;   // desired wall distance (m)
static constexpr double TOL_M           = 0.05;   // no-correction band (m)
static constexpr double FWD_SPEED       = 0.25;   // m/s (forward when aligned)
static constexpr double TURN_RATE       = 1.0;    // rad/s (fixed bang-bang turn)
static constexpr double BIAS_MAG        = 0.35;   // how strongly to bias toward/away from wall (0..1)
static constexpr double ANG_OK_RAD      = 15.0 * M_PI / 180.0; // if desired dir within ±15°, go straight
static constexpr int    LOOP_HZ         = 15;     // control loop
// --------------------------------

// If your API does NOT provide angles with the scan, we synthesize [-pi, pi]
static void ensureAngles(std::vector<float>& angles, int N) {
    if (static_cast<int>(angles.size()) == N) return;
    angles.resize(N);
    if (N <= 1) { angles[0] = 0.0f; return; }
    const float start = -static_cast<float>(M_PI);
    const float step = (2.0f * static_cast<float>(M_PI)) / static_cast<float>(N-1);
    for (int i = 0; i < N; ++i) {
        angles[i] = start + i*step;
    }
}

static void normalize2D(double& x, double& y) {
    const double n = std::hypot(x, y);
    if (n > 1e-9) { x /= n; y /= n; }
}

// Bang-bang steering decision:
// - Compute desired direction vector v (unit) in robot frame.
// - If |heading_angle(v)| <= ANG_OK_RAD -> drive forward
// - Else turn left/right at fixed TURN_RATE (vy=0 for diff-drive).
static void bangBangDrive(Robot& robot, double vx_des, double vdir_x, double vdir_y) {
    // Angle between robot forward [1,0] and v = atan2(vy, vx)
    const double ang = std::atan2(vdir_y, vdir_x);

    if (std::fabs(ang) <= ANG_OK_RAD) {
        // Mostly straight in front -> go forward
        robot.drive(vx_des, /*vy=*/0.0, /*wz=*/0.0);
    } else {
        const double wz = (ang > 0.0) ? +TURN_RATE : -TURN_RATE;
        // When turning, keep a small crawl forward to avoid stalling.
        robot.drive(0.15, /*vy=*/0.0, wz);
    }
}

int main() {
    Robot robot;  // adapt if your constructor differs

    const double dt = 1.0 / static_cast<double>(LOOP_HZ);
    std::vector<float> ranges, angles;

    // Fixed world Z axis (for cross product to rotate 90° in the plane)
    const std::vector<double> k_hat{0.0, 0.0, 1.0};

    while (true) {
        // 1) Read scan
        // If your API is robot.readLidarScan(ranges) only, comment the next line and call the one-arg version.
        bool ok = robot.readLidarScan(ranges, angles); // returns true/false in many APIs
        if (!ok) {
            // Fallback if your API returns void:
            // robot.readLidarScan(ranges, angles);
            // or robot.readLidarScan(ranges); ensureAngles(angles, ranges.size());
        }
        if (angles.size() != ranges.size()) {
            ensureAngles(angles, static_cast<int>(ranges.size()));
        }
        if (ranges.empty()) {
            // No data — stop as a safety
            robot.drive(0.0, 0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 2) Find nearest wall ray
        int i_min = -1;
        try {
            i_min = findMinDist(ranges);
        } catch (...) {
            // No valid rays — stop
            robot.drive(0.0, 0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        const double r_min   = static_cast<double>(ranges[i_min]);
        const double th_wall = static_cast<double>(angles[i_min]); // angle where wall is closest

        // 3) Build normal and tangent using cross product
        //    Normal points *from robot toward wall* along the shortest ray.
        double nx = std::cos(th_wall);
        double ny = std::sin(th_wall);
        std::vector<double> n_vec{nx, ny, 0.0};

        // Tangent along wall is k_hat x n (90° CCW rotation of n in the plane)
        std::vector<double> t_vec = crossProduct(k_hat, n_vec);  // [-ny, nx, 0]
        double tx = t_vec[0], ty = t_vec[1];
        normalize2D(tx, ty);

        // 4) Bang-bang distance correction:
        //    If too far from wall, bias toward the wall (subtract normal).
        //    If too close, bias away (add normal).
        double vx = tx, vy = ty; // start with tangent (go along the wall)

        if (r_min > SETPOINT_M + TOL_M) {
            // too far -> bias toward wall
            vx -= BIAS_MAG * nx;
            vy -= BIAS_MAG * ny;
        } else if (r_min < SETPOINT_M - TOL_M) {
            // too close -> bias away from wall
            vx += BIAS_MAG * nx;
            vy += BIAS_MAG * ny;
        }
        normalize2D(vx, vy);

        // 5) Convert desired direction (vx, vy) into diff-drive command via bang-bang steering
        bangBangDrive(robot, /*vx_des=*/FWD_SPEED, vx, vy);

        // 6) Loop timing
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    return 0;
}
