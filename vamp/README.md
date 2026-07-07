# Tabletop Pick Demo

This demonstration plans a collision-free path for a Franka Emika Panda arm to approach a target cylinder standing on a table.

## Installation

Ensure you have activated the workspace python virtual environment:

```sh
source env/bin/activate
```

## Running the Demo

1. **Plan the Trajectory**:
   Run the planning script to compute a collision-free path using OMPL and VAMP:
   ```sh
   python vamp/pick.py
   ```
   This will output:
   * `vamp/trajectory_data.json`

2. **Visualize the Path**:
   Open **[vamp/visualizer.html](visualizer.html)** directly in your browser. Drag and drop the generated `vamp/trajectory_data.json` file into the uploader interface to watch the 3D playback.

3. **Visualize the Static Scene**:
   Open **[vamp/scene_visualizer.html](scene_visualizer.html)** directly in your browser to view the static tabletop environment and check static poses.
