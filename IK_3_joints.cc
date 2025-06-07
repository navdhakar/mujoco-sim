#include <stdio.h>
#include <math.h>

// Link lengths in meters

// these are approximate linkage lengths in cassie.
// left-hip(thigh): 0.09
//left shin: 0.4351m
//left tarsus: 0.4207 m
#define L1 0.01
#define L2 0.09
#define L3 0.435
#define L4 0.420

#define a 0.1
#define b 0.09
#define c 0.435
#define d 0.420

#define hip_x 0.0210
#define hip_y 0.1350
#define hip_z 1.0096


#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
int inverseKinematics3Link(double x, double y, double z, double* hip_roll, double* hip_yaw, double* hip_pitch, double* knee, double* foot) {
    // Step 1: Compute hip roll and yaw from x and y
    *hip_yaw = atan2(y, x);   // Rotation around z-axis
    double r_xy = sqrt(x * x + y * y);
    *hip_roll = atan2(y, z);  // Rotation around x-axis

    // Step 2: Convert to 2D IK in sagittal plane
    double r = sqrt(r_xy * r_xy + z * z);
    double cos_knee = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (cos_knee < -1.0) cos_knee = -1.0;
    if (cos_knee > 1.0) cos_knee = 1.0;
    *knee = acos(cos_knee);

    double angle_a = atan2(z, r_xy);
    double angle_b = acos((L1*L1 + r*r - L2*L2)/(2*L1*r));
    *hip_pitch = angle_a + angle_b;

    // Optional: foot angle correction
    *foot = -(*hip_pitch + *knee);
    return 0;
}
int IK5Dof(double x, double y, double z, double* hip_roll, double* hip_yaw, double* hip_pitch, double* knee, double* foot){
    double L = sqrt((x*x - hip_x*hip_x) + (y*y - hip_y*hip_y) + (z*z - hip_z*hip_z));
    double m = sqrt(L-a*a);
    double L_psi = sqrt(x*x + y*y + (z*z - (d*d - y*y)));
    double L_phi = sqrt(x*x + y*y + (z*z -(-(b*b - y*y))));
    double psi = sqrt(L_psi - (a * a));
    double phi = sqrt(L_phi - (a * a));
    *hip_pitch = DEG2RAD(90) - asin(y/psi) - acos(((b*b) - (a*a) - (c*c) + L_psi)/2*b*psi); // theta hip
    *hip_yaw = 0; // we are not considering this DOF.
    *hip_roll = atan(x/z) + acos(a/sqrt(x*x + z*z)) - DEG2RAD(90); // theta hip abad
    printf("\n%.2f\n", atan(x/z));
    printf("\n%.2f\n", acos(a/sqrt(x*x + z*z)));
    *knee = acos(((b*b) + (c*c) - psi)/2*b*c);
    *foot = acos(((c*c) + (d*d) - phi)/2*c*d);
    return 0;
}
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
    double x = 0.4, y = 0.0, z = 1.0;  // Desired foot position in meters

    double hip_roll, hip_yaw, hip_pitch, knee, foot;
    // if (cassie_leg_ik(x, y, z, &hip_roll, &hip_yaw, &hip_pitch, &knee, &foot) == 0) {
    //     printf("cassie_leg_ik:\n");
    //     printf("Hip Roll   : %.2f \n", hip_roll);
    //     printf("Hip Yaw    : %.2f \n", hip_yaw);
    //     printf("Hip Pitch  : %.2f \n", hip_pitch);
    //     printf("Knee       : %.2f \n", knee);
    //     printf("Foot       : %.2f \n", foot);
    // } else {
    //     printf("No solution found for the given position.\n");
    // }
    if (inverseKinematics3Link(x, y, z, &hip_roll, &hip_yaw, &hip_pitch, &knee, &foot) == 0) {
        printf("inverseKinematics3Link Solution:\n");
        printf("Hip Roll   : %.2f \n", hip_roll);
        printf("Hip Yaw    : %.2f \n", hip_yaw);
        printf("Hip Pitch  : %.2f \n", hip_pitch);
        printf("Knee       : %.2f \n", knee);
        printf("Foot       : %.2f \n", foot);
    } else {
        printf("No solution found forinverseKinematics3Link for the given position.\n");
    }
     if (IK5Dof(x, y, z, &hip_roll, &hip_yaw, &hip_pitch, &knee, &foot) == 0) {
        printf("IK5Dof Solution:\n");
        printf("Hip Roll   : %.2f \n", hip_roll);
        printf("Hip Yaw    : %.2f \n", hip_yaw);
        printf("Hip Pitch  : %.2f \n", hip_pitch);
        printf("Knee       : %.2f \n", knee);
        printf("Foot       : %.2f \n", foot);
    } else {
        printf("No solution found for IK5Dof for the given position.\n");
    }

    return 0;
}

