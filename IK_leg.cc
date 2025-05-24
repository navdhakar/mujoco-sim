#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define L1 0.1201  // Length of link 1 (in meters)
#define L2 0.0770  // Length of link 2 (in meters)

typedef struct {
    bool reachable;
    double hip_roll;   // in radians
    double hip_pitch;  // in radians
    double knee;       // in radians
} IKResult;

IKResult inverse_kinematics(double x, double y, double z) {
    IKResult result;

    // Compute hip roll (rotation around X axis)
    result.hip_roll = atan2(y, -z);
    double z_proj = sqrt(y * y + z * z);

    // Distance to target in x-z_proj plane
    double d = sqrt(x * x + z_proj * z_proj);

    // Check reachability
    if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        result.reachable = false;
        return result;
    }

    // Law of cosines for knee angle
    double cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2);
    if (cos_knee < -1.0 || cos_knee > 1.0) {
        result.reachable = false;
        return result;
    }

    double knee_angle = M_PI - acos(cos_knee);

    // Angle from hip to foot
    double alpha = atan2(z_proj, x);

    // Law of cosines for angle at hip
    double cos_beta = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
    double beta = acos(cos_beta);

    double hip_pitch = alpha - beta;

    result.reachable = true;
    result.hip_pitch = hip_pitch;
    result.knee = knee_angle;

    return result;
}

int main() {
    double x, y, z;
    printf("Enter target foot position (x y z in meters): ");
    scanf("%lf %lf %lf", &x, &y, &z);

    IKResult res = inverse_kinematics(x, y, z);

    if (res.reachable) {
        printf("Reachable!\n");
        printf("Hip Roll:   %.6f radians\n", res.hip_roll);
        printf("Hip Pitch:  %.6f radians\n", res.hip_pitch);
        printf("Knee:       %.6f radians\n", res.knee);
    } else {
        printf("Target is unreachable with the given link lengths.\n");
    }

    return 0;
}

