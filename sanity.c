#include "C:\Users\navde\OneDrive\Documents\dev\mujoco-3.3.2-windows-x86_64\include\mujoco\mujoco.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>  // for Sleep()

char error[1000];
mjModel* m;
mjData* d;

int main(void) {
  // activate MuJoCo license if needed (not for >= 2.3.0)
  // mj_activate("mjkey.txt");

  // load model from file and check for errors
  m = mj_loadXML("arm26.xml", NULL, error, 1000);
  if (!m) {
    printf("Error loading model: %s\n", error);
    return 1;
  }

  // make data corresponding to model
  d = mj_makeData(m);
  printf("Hello from MuJoCo simulation!\n");
  printf("Starting MuJoCo...\n");
  printf("Initial time: %f\n", d->time);

  // run simulation for 10 seconds
  double print_interval = 0.1;
  double next_print = 0.0;

  while (d->time < 10) {
    mj_step(m, d);

    // print joint positions and velocities every 0.1s
    if (d->time >= next_print) {
      printf("Time: %.3f\n", d->time);
      for (int i = 0; i < m->nq; i++) {
        printf("  qpos[%d]: %f\n", i, d->qpos[i]);
      }
      for (int i = 0; i < m->nv; i++) {
        printf("  qvel[%d]: %f\n", i, d->qvel[i]);
      }
      printf("\n");
      next_print += print_interval;
      Sleep(50);  // slow down console output
    }
  }

  // free model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  return 0;
}
