# Robot Arm - Soda Grasp

Human-robot interaction is a major field of study that tries to bridge the gap between the human and the machine world. The following project implements an algorithm that uses multiple robotics algorithm to accomplish the task of grabbing and lifting a soda can.

The project implements a rrt algorithm that finds a path from the initial starting position of the robotic arm to a pre-determined position of the soda can. 

The algorithm is provided with the bounds of freedom matrix for grabbing the soda can. Once a feasible path is found the arm traverses the path to the soda can grasps it.

![Soda can and robotic arm - with valid grasping conditions](https://github.com/vakharia-aarya/soda-grasp/blob/main/tsr_vis_2.png)

The image above displays the soda can and the robotic arm; the red, blue and green pipes display the valid bounds for grasping the can.


Once the soda can is grasped the rrt is re-run to position the can above the bowl. Once its positioned, the arm rotates in order to pour the liquid into the bowl. 

![The Soda can being grasped and lifted](https://github.com/vakharia-aarya/soda-grasp/blob/main/jac_vis_1.png)
