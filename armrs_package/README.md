# armrs_package
Code repository for cooperative control of multi-robot system

To be used in Aerial Robotics and Multi-Robot System Course


<!-- ## 
Each of the file describes the components for easier separation of algorithm in python, ROS wrapper, and real-robot implementation. Thus, ease the process of transition from proof-of-concept to verification

- yaml files: contain simulation-specific setups and scenario-specific parameters
- yaml_loader: parse the yaml files and store them into the designated class for easier access and categorization
- simulator.py: main calculation to model the movement of the robots based on the control input
- visualizer.py: basic encapsulation of visualizing the movement and range-sensor measurement of the robots
- main_controller.py: contain the encapsulation of estimation and controller calculation for each robot
 -->

## Setup in ROS2 HUMBLE
A recording on how to setup the package is available here: [External link to sharepoint](https://utufi-my.sharepoint.com/personal/mwatma_utu_fi/_layouts/15/stream.aspx?id=%2Fpersonal%2Fmwatma%5Futu%5Ffi%2FDocuments%2FToShare%2FARMRS%5FLecture3%5FRecording%2FSimulationTool%5FExplanation%2Emp4&ga=1&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2Ec55085ea%2D547e%2D455f%2D8e4b%2D3f6e3fde763a)

But the basic installation setup is summarized in these bullet points:
- install dependency `pip3 install pyyaml`
- clone this folder inside `~/ros2_ws/src`
- also clone the armrs_msgs from the following repository: https://github.com/mwsatman/armrs_msgs
- From `~/ros2_ws` build your workspace `colcon build --symlink-install`
- Source the compiled code via `source install/setup.bash`

- on ROS2_sim_launch, remember to correct the absolute path of the yaml file.
- run the simulation via `ros2 launch mrs_cbf_journal ROS2_sim_launch.py`


## NOTES on using the library

### Getting the neighbor data
The simulation template already implemented communication exchange via `StateExchange.msg`. 
The communication is assumed to be undirected (two-way communication). 
The communication links are dictated by the adjacency matrix within each scenario yaml (e.g., see `adjacency_mat` at `scenario_demo_1formation.yaml`).
Within every node, the adjacency matrix is then parsed by `yaml_loader.py` and then used to subscribe to the respective neighbor robots' message.

Specific to controller implementation (`main_controller.py` and `ROS2_dist_controller.py`), 
the template already implement the message exchange and parsing on neighbor robot's pose and then put them inside the `Estimation` class. 
Then, accessing this information inside the `Controller` class can be done as follows 
```python
for j_id in estimation_dict.neighbours_data:
    j_pos = estimation_dict.neighbours_data[j_id]['pos'] # robot's center position x, y, and z = 0
    j_theta = estimation_dict.neighbours_data[j_id]['theta'] # angle theta from x-axis world reference
    j_lahead = estimation_dict.neighbours_data[j_id]['lahead'] # look-ahead position

    # continue with implementing cooperative control for each neighbour data
```

Exchanging other data require update on the following:
- create new variables inside `StateExchange.msg`
- inside `ROS2_dist_controller.py`, construct and parse the new variables to `StateExchange.msg`
- Then pass the data between `ROS2_dist_controller.py` and `main_controller.py`, e.g., via `Estimation` or `Controller` class 


### Plotting Voronoi cells

Visualizing voronoi cells can be done by updating the following, inside the `vis_loop` function in `ROS2_visualizer.py`

```python
    def vis_loop(self):
        # .... other existing code

        # Update the plot
        elapsed_time = (now - self.start_t)
        self.plot_vis.update(elapsed_time, self.robot_est, self.evaluator)


        # --------------- Plot VORONOI Cells ----------------
        # remember to add the needed calculation to setup the list of robot id
        # and the vertices of the voronoi cell

        for id in list_of_robot_id_voronoi:
            self.plot_vis.plot_voronoi_cell(id, vertices_voronoi_cell)
        

        # .... continue with other existing code

```
