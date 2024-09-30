# FA-RRT\*N Path Planning and Control for Autonomous Vehicles

This project presents an implementation and evaluation of the Fuzzy Adaptive RRT*N path planning algorithm
for autonomous vehicles in the CARLA simulator. The report is
based on the Intelligent Adaptive RRT* algorithm proposed in
the paper ”Intelligent Adaptive RRT* Path Planning Algorithm
for Mobile Robots” by Omar et al. We replicated the FARRT*N algorithm and implemented it with autonomous vehicles
navigating environments (Towns) in CARLA. The main goal
of the Fuzzy Adaptive RRT*N is to improve the efficiency of
path generation in terms of both computation time and path
quality. It achieves this by incorporating fuzzy logic to adapt
the RRT* algorithm parameters dynamically based on the local
environment characteristics. Specifically, it adjusts the step size
and goal biasing parameters using fuzzy rules that consider obstacle density and distance to the goal. In our implementation, we
developed the algorithm from scratch in Python and integrated
it with the CARLA client to control the autonomous vehicle.
We evaluated its performance by conducting multiple runs with
the same conditions and compared the results of RRT* and FARRT*N. Results show that the FA-RRT*N approach generates
high-quality, collision-free paths while reducing computation
times compared to the original RRT* algorithm

---

### Running the Algorithms

To execute the path planning algorithms, follow these steps:

- **FA-RRT\*N Algorithm**:  
  Run the Python script `FA-RRT_star_N.py` in your preferred code editor.
  ```bash
  python FA-RRT_star_N.py
  ```

- **RRT\* Algorithm**:  
  Run the Python script `RRT_star.py` in your preferred code editor.
  ```bash
  python RRT_star.py
  ```

---

### System Requirements for Simulation

- **Minimum RAM**: 8 GB  
- **Graphics Card**: 4 GB  
- **Storage**: At least 60 GB

---

### Visualizing the Simulation

To visualize the simulation in the CARLA simulator:

1. Install the CARLA simulator on your device.
2. Locate and run the `CarlaUE4.exe` file, typically found at:  
   `<path_to_carla_installation>\CARLA_Latest\WindowsNoEditor\`
3. Open PowerShell as an administrator and execute the following script:
   ```bash
   python FA-RRT_star_N_CARLA.py
   ```
4. Ensure the CARLA server is fully initialized before running the script (this may take a few moments).

This will launch a Pygame window where you can visualize the simulation within the CARLA environment.

---

### Simulation Video

Watch the simulation in action here:  
[Simulation Video](https://youtu.be/uBteTbpqdzo)
