#include <stdio.h>
#include <math.h>

// Link lengths in meters

// these are approximate linkage lengths in cassie.
// left-hip(thigh): 0.09
//left shin: 0.4351m
//left tarsus: 0.4207 m
#define L1 0.09
#define L2 0.435
#define L3 0.420

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// Inverse Kinematics for Cassie's left leg
int cassie_leg_ik(double x, double y, double z, 
                  double* hip_roll, double* hip_yaw, 
                  double* hip_pitch, double* knee, double* foot) {
    
    // Compute hip roll and yaw
    *hip_roll = atan2(z, y);  // roll: rotation around x
    *hip_yaw  = atan2(x, y);  // yaw: rotation around z

    // Project foot position into sagittal (x-y) plane after correcting hip_roll
    double xy_dist = sqrt(x*x + y*y);
    double leg_plane_x = xy_dist;
    double leg_plane_z = z;

    // Distance from hip to foot
    double D = sqrt(leg_plane_x*leg_plane_x + leg_plane_z*leg_plane_z);

    // Check reachability
    if (D > (L1 + L2)) {
        printf("Target out of reach.\n");
        return -1;
    }

    // Law of Cosines for knee angle
    double cos_knee = (L1*L1 + L2*L2 - D*D) / (2 * L1 * L2);
    *knee = M_PI - acos(cos_knee);  // inward knee bend

    // Compute hip pitch
    double alpha = atan2(leg_plane_z, leg_plane_x);
    double cos_phi = (L1*L1 + D*D - L2*L2) / (2 * L1 * D);
    double phi = acos(cos_phi);
    *hip_pitch = alpha - phi;

    // Foot pitch to keep foot flat
    *foot = -(*hip_pitch + *knee);

    return 0;
}

int main() {
    double x = 0.2, y = 0.05, z = -0.4;  // Desired foot position in meters

    double hip_roll, hip_yaw, hip_pitch, knee, foot;
    if (cassie_leg_ik(x, y, z, &hip_roll, &hip_yaw, &hip_pitch, &knee, &foot) == 0) {
        printf("IK Solution:\n");
        printf("Hip Roll   : %.2f \n", hip_roll);
        printf("Hip Yaw    : %.2f \n", hip_yaw);
        printf("Hip Pitch  : %.2f \n", hip_pitch);
        printf("Knee       : %.2f \n", knee);
        printf("Foot       : %.2f \n", foot);
    } else {
        printf("No solution found for the given position.\n");
    }

    return 0;
}

