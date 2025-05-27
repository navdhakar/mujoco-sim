model:= model_file


# simple sanity to check if the state are genereted or not.
sanity_check:sanity
	./sanity.exe

sanity: sanity.c
	gcc sanity.c -IC:\Users\navde\OneDrive\Documents\dev\mujoco-3.3.2-windows-x86_64\include -LC:\Users\navde\OneDrive\Documents\dev\mujoco-3.3.2-windows-x86_64\bin -lmujoco -o sanity

# to test open gl graphics render for sim
graphics_check:basic
	./basic.exe hello.xml
basic:basic.cc
	cc basic.cc   -IC:/Users/navde/OneDrive/Documents/dev/mujoco-3.3.2-windows-x86_64/include   -IC:/msys64/ucrt64/include   -LC:/Users/navde/OneDrive/Documents/dev/mujoco-3.3.2-windows-x86_64/bin   -LC:/msys64/ucrt64/lib   -lmujoco -lglfw3 -lopengl32 -lm -luser32 -lgdi32 -o basic
	
cassie_run:cassie
	./cassie_sim.exe $(model)
cassie:cassie.cc
	cc cassie.cc -IC:/Users/navde/OneDrive/Documents/dev/mujoco-3.3.2-windows-x86_64/include   -IC:/msys64/ucrt64/include   -LC:/Users/navde/OneDrive/Documents/dev/mujoco-3.3.2-windows-x86_64/bin   -LC:/msys64/ucrt64/lib   -lmujoco -lglfw3 -lopengl32 -lm -luser32 -lgdi32 -o cassie_sim

# this is to test 3 joints IK equations.
ik_bi_run:ik_3_joints
	./ik_3_joints
	
ik_3_joints:ik_3_joints.cc
	gcc ik_3_joints.cc -o ik_3_joints -lm
