#!/usr/bin/env python

import time

import adapy
import numpy as np
import rospy


class AdaRRT():
    """
    Rapidly-Exploring Random Trees (RRT) for the ADA controller.
    """
    joint_lower_limits = np.array([-3.14, 1.57, 0.33, -3.14, 0, 0])
    joint_upper_limits = np.array([3.14, 5.00, 5.00, 3.14, 3.14, 3.14])

    class Node():
        """
        A node for a doubly-linked tree structure.
        """
        def __init__(self, state, parent):
            """
            :param state: np.array of a state in the search space.
            :param parent: parent Node object.
            """
            self.state = np.asarray(state)
            self.parent = parent
            self.children = []

        def __iter__(self):
            """
            Breadth-first iterator.
            """
            nodelist = [self]
            while nodelist:
                node = nodelist.pop(0)
                nodelist.extend(node.children)
                yield node

        def __repr__(self):
            return 'Node({})'.format(', '.join(map(str, self.state)))

        def add_child(self, state):
            """
            Adds a new child at the given state.

            :param state: np.array of new child node's statee
            :returns: child Node object.
            """
            child = AdaRRT.Node(state=state, parent=self)
            self.children.append(child)
            return child

    def __init__(self,
                 start_state,
                 goal_state,
                 ada,
                 joint_lower_limits=None,
                 joint_upper_limits=None,
                 ada_collision_constraint=None,
                 step_size=0.25,
                 goal_precision=1.0,
                 max_iter=10000):
        """
        :param start_state: Array representing the starting state.
        :param goal_state: Array representing the goal state.
        :param ada: libADA instance.
        :param joint_lower_limits: List of lower bounds of each joint.
        :param joint_upper_limits: List of upper bounds of each joint.
        :param ada_collision_constraint: Collision constraint object.
        :param step_size: Distance between nodes in the RRT.
        :param goal_precision: Maximum distance between RRT and goal before
            declaring completion.
        :param sample_near_goal_prob:
        :param sample_near_goal_range:
        :param max_iter: Maximum number of iterations to run the RRT before
            failure.
        """
        self.start = AdaRRT.Node(start_state, None)
        self.goal = AdaRRT.Node(goal_state, None)
        self.ada = ada
        self.joint_lower_limits = joint_lower_limits or AdaRRT.joint_lower_limits
        self.joint_upper_limits = joint_upper_limits or AdaRRT.joint_upper_limits
        self.ada_collision_constraint = ada_collision_constraint
        self.step_size = step_size
        self.goal_precision = goal_precision
        self.max_iter = max_iter

    def build(self):
        """
        Build an RRT.

        In each step of the RRT:
            1. Sample a random point.
            2. Find its nearest neighbor.
            3. Attempt to create a new node in the direction of sample from its
                nearest neighbor.
            4. If we have created a new node, check for completion.

        Once the RRT is complete, add the goal node to the RRT and build a path
        from start to goal.

        :returns: A list of states that create a path from start to
            goal on success. On failure, returns None.
        """
        for k in range(self.max_iter):
            # FILL in your code here
            s = np.random.uniform(0,1)
            # s = 1
            # p_sample=self._get_random_sample()
            if s<=0.8:
                p_sample=self._get_random_sample()
            else:
                p_sample=self._get_random_sample_near_goal()
            
            # get random sample near goal with probability 0.2 and
            # get random sample with probability 0.8. Reduce e to 0.2.
            
            current_minDis_node=self._get_nearest_neighbor(p_sample)
            new_node=self._extend_sample(p_sample,current_minDis_node)

            if new_node and self._check_for_completion(new_node):
                # FILL in your code here
                self.goal.parent=new_node
                new_node.add_child(self.goal)

                path=self._trace_path_from_start(self.goal)

                return path

        print("Failed to find path from {0} to {1} after {2} iterations!".format(
            self.start.state, self.goal.state, self.max_iter))

    def _get_random_sample(self):
        """
        Uniformly samples the search space.

        :returns: A vector representing a randomly sampled point in the search
            space.
        """
        # FILL in your code here
        p_sample=[]
        for index in range(len(self.joint_lower_limits)):
            x = np.random.uniform(self.joint_lower_limits[index], self.joint_upper_limits[index])
            p_sample=np.append(p_sample,x)

        return p_sample
    
    def _get_random_sample_near_goal(self):
        # that generates a sample around the goal within
        # a distance of 0.05 along each axis of the search space. 
        distance=0.05
        p_sample=[]
        for index in range(len(self.goal.state)):
            x = np.random.uniform(self.goal.state[index]-distance, self.goal.state[index]+distance)
            p_sample=np.append(p_sample,x)

        return p_sample

    def _get_nearest_neighbor(self, sample):
        """
        Finds the closest node to the given sample in the search space,
        excluding the goal node.

        :param sample: The target point to find the closest neighbor to.
        :returns: A Node object for the closest neighbor.
        """
        # FILL in your code here
        current_node=self.start
        iter = current_node.__iter__()
        # next(iter)
        min_dist=np.inf
        while True:
            # current_state=next(iter).state
            current_node=(next(iter, None))
            
            if not current_node is None:
                current_state=current_node.state
                current_dist=np.linalg.norm(current_state-sample)
                if current_dist<min_dist:
                    min_dist=current_dist
                    current_minDis_state=current_state
                    current_minDis_node=current_node
            else:
                break
        # return current_minDis_state
        return current_minDis_node

    def _extend_sample(self, sample, neighbor):
        """
        Adds a new node to the RRT between neighbor and sample, at a distance
        step_size away from neighbor. The new node is only created if it will
        not collide with any of the collision objects (see
        RRT._check_for_collision)

        :param sample: target point
        :param neighbor: closest existing node to sample
        :returns: The new Node object. On failure (collision), returns None.
        """
        # FILL in your code here
        direction_vector = sample-neighbor.state
        # print(sample)
        # print(neighbor.state)
        # if sample!=neighbor.state:
        if sum(direction_vector)!=0:
            unit_direction_vector=direction_vector/np.linalg.norm(direction_vector)
            new_minDis_state=neighbor.state+self.step_size*unit_direction_vector

            # print("neighbor.state")
            # print(neighbor.state)

            # print("self.step_size")
            # print(self.step_size)

            # print("unit_direction_vector")
            # print(unit_direction_vector)

            # print("new_minDis_state")
            # print(new_minDis_state)

            # If temp is True, then the new node is not added to the tree
            temp=self._check_for_collision(new_minDis_state)
            if temp==True:
                # RRT.Node()
                return None
            else:
                neighbor.add_child(new_minDis_state)
                return AdaRRT.Node(new_minDis_state, neighbor)
        # if sample==neighbor.state:
        else:
            return None

    def _check_for_completion(self, node):
        """
        Check whether node is within self.self.goal_precision distance of the goal.

        :param node: The target Node
        :returns: Boolean indicating node is close enough for completion.
        """
        # FILL in your code here
        # print("_check_for_completion")
        if np.linalg.norm(node.state-self.goal.state)<self.goal_precision:
            return True
        else:
            return False

    def _trace_path_from_start(self, node=None):
        """
        Traces a path from start to node, if provided, or the goal otherwise.

        :param node: The target Node at the end of the path. Defaults to
            self.goal
        :returns: A list of states (not Nodes!) beginning at the start state and
            ending at the goal state.
        """
        # FILL in your code here
        # print("_trace_path_from_start")
        if node==None:
            node=self.goal
        rev_list=[]
        nonrev_list=[]
        # print(node.state)
        while node.parent is not None:
            rev_list.append(node.state)
            node=node.parent
        rev_list.append(node.state)
        while len(rev_list)>0:
            nonrev_list.append(rev_list.pop())
        
        
        return nonrev_list

    def _check_for_collision(self, sample):
        """
        Checks if a sample point is in collision with any collision object.

        :returns: A boolean value indicating that sample is in collision.
        """
        if self.ada_collision_constraint is None:
            return False
        return not self.ada_collision_constraint.is_satisfied(
            self.ada.get_arm_state_space(),
            self.ada.get_arm_skeleton(), sample)


def main():
    sim = True

    # instantiate an ada
    ada = adapy.Ada(True)

    armHome = [-1.5, 3.22, 1.23, -2.19, 1.8, 1.2]
    goalConfig = [-1.72, 4.44, 2.02, -2.04, 2.66, 1.39]
    delta = 0.25
    eps = 1.0

    if sim:
        ada.set_positions(goalConfig)

    # launch viewer
    viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
    world = ada.get_world()

    # add objects to world
    soda_pose = np.eye(4)
    soda_pose[0, 3] = 0.25
    soda_pose[1, 3] = -0.35
    sodaURDFUri = "package://pr_assets/data/objects/can.urdf"
    can = world.add_body_from_urdf_matrix(sodaURDFUri, soda_pose)

    tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
    # x, y, z, rw, rx, ry, rz
    table_pose = [0.3, 0.0, -0.75, 0.707107, 0., 0., 0.707107]
    table = world.add_body_from_urdf(tableURDFUri, table_pose)

    # add collision constraints
    collision_free_constraint = ada.set_up_collision_detection(
            ada.get_arm_state_space(),
            ada.get_arm_skeleton(),
            [can, table])
    full_collision_constraint = ada.get_full_collision_constraint(
            ada.get_arm_state_space(),
            ada.get_arm_skeleton(),
            collision_free_constraint)

    # easy goal
    adaRRT = AdaRRT(
        start_state=np.array(armHome),
        goal_state=np.array(goalConfig),
        ada=ada,
        ada_collision_constraint=full_collision_constraint,
        step_size=delta,
        goal_precision=eps)

    rospy.sleep(1.0)

    path = adaRRT.build()
    if path is not None:
        print("Path waypoints:")
        print(np.asarray(path))
        waypoints = []
        for i, waypoint in enumerate(path):
            waypoints.append((0.0 + i, waypoint))

        t0 = time.clock()
        traj = ada.compute_joint_space_path(
            ada.get_arm_state_space(), waypoints)
        t = time.clock() - t0
        print(str(t) + "seconds elapsed")
        raw_input('Press ENTER to execute trajectory and exit')
        ada.execute_trajectory(traj)
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
