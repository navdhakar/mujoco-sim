// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <time.h>
#include <windows.h> 

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
void inverseKinematicsToXYZ(mjModel* m, mjData* d, const char* end_effector_body_name, const mjtNum target[3]) {
  int body_id = mj_name2id(m, mjOBJ_BODY, end_effector_body_name);
  if (body_id == -1) {
    printf("Body %s not found!\n", end_effector_body_name);
    return;
  }

  // Joint names of left leg to update
  const char* joint_names[] = {
    "left-hip-roll",
    "left-hip-yaw",
    "left-hip-pitch",
    "left-knee"
  };
  const int njoints = sizeof(joint_names) / sizeof(joint_names[0]);
  
  int joint_ids[njoints];
  for (int i = 0; i < njoints; ++i) {
    joint_ids[i] = mj_name2id(m, mjOBJ_JOINT, joint_names[i]);
    if (joint_ids[i] == -1) {
      printf("Joint %s not found!\n", joint_names[i]);
      return;
    }
  }

  const double tol = 1e-3;
  const double alpha = 0.01;
  const int max_iter = 100;

  mjtNum pos[3];
  mjtNum jac[3 * m->nv];  // Jacobian for all dofs

  for (int iter = 0; iter < max_iter; ++iter) {
    mj_forward(m, d);

    // Get current foot position
    mju_copy3(pos, d->xpos + 3 * body_id);

    // Compute error vector
    mjtNum err[3] = {
      target[0] - pos[0],
      target[1] - pos[1],
      target[2] - pos[2]
    };

    double err_norm = mju_norm3(err);
    if (err_norm < tol) {
      printf("IK converged in %d iterations\n", iter);
      return;
    }

    // Compute Jacobian for body COM
    mju_zero(jac, 3 * m->nv);
    mj_jacBodyCom(m, d, jac, NULL, body_id);

    // Update only left leg joints using gradient descent on qpos
    for (int j = 0; j < njoints; ++j) {
      int joint_id = joint_ids[j];
      double grad = 0;
      // jacobian matrix is 3 x nv, index by row * nv + col
      for (int k = 0; k < 3; ++k) {
        grad += jac[k * m->nv + joint_id] * err[k];
      }
      d->qpos[joint_id] += alpha * grad;

      // Clamp qpos within joint range
      if (d->qpos[joint_id] < m->jnt_range[2 * joint_id]) {
        d->qpos[joint_id] = m->jnt_range[2 * joint_id];
      }
      if (d->qpos[joint_id] > m->jnt_range[2 * joint_id + 1]) {
        d->qpos[joint_id] = m->jnt_range[2 * joint_id + 1];
      }
    }
  }

  printf("IK did not converge.\n");
}



// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=2) {
    std::printf(" USAGE:  basic modelfile\n");
    return 0;
  }

  // load and compile model
  char error[1000] = " ";
  if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    double print_interval = 0.1;
    double next_print = 0.0;
    mjtNum simstart = d->time;
    // Target foot position (in world coordinates)
    mjtNum target[3] = {0.2, 0.05, -0.4};  // Example target (x, y, z)

    // Perform IK for "left-foot"
    // inverseKinematicsToXYZ(m, d, "left-shin", target);
     int left_knee_act_id = mj_name2id(m, mjOBJ_ACTUATOR, "left-knee");
     int hip_pitch_act_id = mj_name2id(m, mjOBJ_ACTUATOR, "left-hip-pitch");
     int hip_roll_act_id = mj_name2id(m, mjOBJ_ACTUATOR, "left-hip-roll");
     int hip_yaw_act_id = mj_name2id(m, mjOBJ_ACTUATOR, "left-hip-yaw");
     int left_foot_act_id = mj_name2id(m, mjOBJ_ACTUATOR, "left-foot");
     
     
     int body_id = mj_name2id(m, mjOBJ_BODY, "left-foot");
     int sensor_id = mj_name2id(m, mjOBJ_SENSOR, "left-foot-output");
      // d->ctrl[hip_roll_act_id] = 3.02;
      // d->ctrl[hip_yaw_act_id] = 0.24;
      // d->ctrl[hip_pitch_act_id] = 0.21;
      // d->ctrl[left_knee_act_id] = 1.50;
      // d->ctrl[left_foot_act_id] = -1.70;
    
    while (d->time - simstart < 1.0/60.0) {
     
      mj_step(m, d);
      
    }
//     if (sensor_id != -1) {
//     int adr = m->sensor_adr[sensor_id];
//     printf("Left foot joint position sensor = %.6f\n", d->sensordata[adr]);
// } else {
//     printf("Sensor 'left-foot-output' not found!\n");
// }
    // Get and print left foot position (world coordinates)
    // if (d->time >= next_print) {
    //   printf("Time: %.3f\n", d->time);
    //   for (int i = 0; i < m->nq; i++) {
    //     printf("  qpos[%d]: %f\n", i, d->qpos[i]);
    //   }
    //   for (int i = 0; i < m->nv; i++) {
    //     printf("  qvel[%d]: %f\n", i, d->qvel[i]);
    //   }
    //   printf("\n");
    //   next_print += print_interval;
    //   Sleep(50);  // slow down console output
    // }
    mjtNum* foot_pos = d->xpos + 3 * body_id;
    printf("left_foot_id %d", body_id);
    printf("Left foot position: x = %.4f, y = %.4f, z = %.4f\n", foot_pos[0], foot_pos[1], foot_pos[2]);


    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
