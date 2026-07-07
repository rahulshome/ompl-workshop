# OMPL 2.0 Workshop
This repository hosts the demo code for the OMPL 2.0 workshop. The workshop contains two demos, a minimal manipulator planning demo in `ompl_manip` and a SIMD accelerated manipulator planning demo in `vamp`.

## System Requirements
- Linux, MacOS, or Windows with a WSL 2 Ubuntu distribution (Linux and MacOS recommended)
- Python version between 3.11 to 3.13, pip
- git
- Eigen

On WSL/Ubuntu (other distribution package managers offer similar packages)
```bash
sudo apt install libeigen3-dev
```
On MacOS
```bash
brew install eigen
```

## Setup

**1. Clone this repository**

```bash
git clone https://github.com/rahulshome/ompl-workshop.git
cd ompl-workshop
```

**2. Create and source a virtual environment**

```bash
python -m venv env
source env/bin/activate
```

**3. Install dependencies**

```bash
python -m pip install -r requirements.txt
```

## Demos

This repository contains two main workshops:

### 1. Minimal Manipulator Planning (`ompl_manip`)
A minimal manipulator planning demo using OMPL.

### 2. Tabletop Pick Demo (`vamp`)
An accelerated tabletop pick demo using OMPL and VAMP.

* **Plan the path**:
  ```bash
  python vamp/pick.py
  ```
* **Visualize the path**:
  Open **[vamp/visualizer.html](vamp/visualizer.html)** in a web browser, and drag-and-drop the generated `vamp/trajectory_data.json` file into the uploader interface to watch the 3D playback.
* **Visualize the static scene**:
  Open **[vamp/scene_visualizer.html](vamp/scene_visualizer.html)** in a web browser to view the static tabletop environment and check static poses.



### ⚠️ Optional: Rerun Visualization
The minimal manipulator demo (`ompl_manip`) and the original VAMP planning templates use [Rerun](https://rerun.io/) for visualization. Installing Rerun is **optional** and can be skipped if you only plan to run the tabletop pick demo (which uses the browser-based WebGL visualizer).

If you wish to run the Rerun-based demos:
1. **Install Rerun**:
   ```bash
   pip install rerun-sdk
   ```
2. **WSL Vulkan Configuration**:
   It is recommended to set up Vulkan drivers for WSL:
   ```bash
   sudo add-apt-repository ppa:kisak/kisak-mesa
   sudo apt-get update
   sudo apt-get install -y mesa-vulkan-drivers
   ```
   Test your setup with:
   ```bash
   rerun --renderer=vulkan
   ```
   And run those scripts with `WGPU_BACKEND=vulkan`:
   ```bash
   WGPU_BACKEND=vulkan python <script_dir>/<script_name>.py
   ```