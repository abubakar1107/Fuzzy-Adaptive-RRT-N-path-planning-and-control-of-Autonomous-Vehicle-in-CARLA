Here's the markdown code for your README:

```markdown
# FA-RRT\*N Path Planning and Control for Autonomous Vehicles

This project presents the implementation and evaluation of the Fuzzy Adaptive RRT\*N (FA-RRT\*N) path planning algorithm for autonomous vehicles within the CARLA simulator. Based on the "Intelligent Adaptive RRT* Path Planning Algorithm for Mobile Robots" by Omar et al., the FA-RRT\*N algorithm enhances the original RRT\* by using fuzzy logic to dynamically adjust parameters like step size and goal bias based on environmental factors such as obstacle density and distance to the goal. The primary objective of FA-RRT\*N is to optimize path generation in terms of computation time and path quality, offering high-quality, collision-free paths for autonomous vehicle navigation. The algorithm was developed from scratch in Python and integrated with CARLA to simulate autonomous vehicle behavior in different environments. Performance evaluations show that FA-RRT\*N outperforms traditional RRT\* by generating faster, more efficient paths.

---

### Running the Algorithms

To run the path planning algorithms:

- **FA-RRT\*N Algorithm**:  
  Execute the Python script `FA-RRT_star_N.py`:
  ```bash
  python FA-RRT_star_N.py
  ```

- **RRT\* Algorithm**:  
  Execute the Python script `RRT_star.py`:
  ```bash
  python RRT_star.py
  ```

---

### System Requirements for Simulation

- **RAM**: Minimum 8 GB  
- **Graphics Card**: Minimum 4 GB  
- **Storage**: At least 60 GB

---

### Visualizing the Simulation

To visualize the FA-RRT\*N simulation in CARLA:

1. Install the CARLA simulator.
2. Run the `CarlaUE4.exe` file, typically located in:  
   `<path_to_carla_installation>\CARLA_Latest\WindowsNoEditor\`
3. Open PowerShell as an administrator and run the FA-RRT\*N script:
   ```bash
   python FA-RRT_star_N_CARLA.py
   ```
4. Wait for the CARLA server to fully initialize before executing the script.

Once ready, a Pygame window will launch, allowing you to view the simulation within the CARLA environment.

---

### Simulation Video

Watch the FA-RRT\*N simulation in action:  
[Simulation Video](https://youtu.be/uBteTbpqdzo)
```

This code should work perfectly as a markdown file for your project’s README.
