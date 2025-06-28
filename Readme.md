## Multi-Target Tracking ROS1 Integration

This package integrates the [`multi_target_tracking`](multi_target_tracking/Readme.md) library for use with ROS1. Due to ROS1's limitations with modern Python package structures, some manual steps are required after cloning the repository.

## 1. Installation

### 1.1 Clone and Initialize

```bash
git clone <your-repo-url>
cd <your-repo>
git submodule update --init --recursive
```

### 1.2 Build the Workspace

From your catkin workspace root:

```bash
cd /path/to/your/catkin_ws
catkin_make
```

### 1.3 Make Nodes Executable

```bash
cd /path/to/mtt_ros_node
chmod +x nodes/JPDA_node.py
```

### 1.4 Run the Node

```bash
ROS_NAMESPACE=/machine_1 rosrun multi_target_tracking JPDA_node.py \
  _robotID:=1 \
  _numRobots:=3 \
  _config:=/absolute/path/to/multi_target_tracking/mtt_config.yaml
```
\
⚠️ **Performance Note**: As the Tracks are published in real time, saving them in memory is not necessary. 
1. Set the `maximumTrackLength` parameter in [`mtt_config.yaml`](multi_target_tracking/cfg/mtt_config.yaml) to a low value (1 or 2)
2. Set the track status in [`score_logic.py`](multi_target_tracking/track/score_logic.py) and [`mn_logic.py`](multi_target_tracking/track/mn_logic.py) to *DELETED* instead of *ARCHIVED*

## 2. Configuration

Configure tracking parameters in [`multi_target_tracking/cfg/mtt_config.yaml`](multi_target_tracking/cfg/mtt_config.yaml):

```yaml
# Detection probabilities
PD: 0.9      # Probability of detection
PG: 1        # Probability of gating
FAR: 0.01    # False alarm rate

# Gating parameters
thresholdMahalanobis: 3.0  # Threshold for Mahalanobis gating
thresholdID: 0.1           # Threshold for ID gating

# Filter parameters
velocityDecayTime: 20.0    # Time constant for velocity decay in filter
offsetDecayTime: 30.0      # Time constant for offset decay in filter

# Track management
maximumTrackLength: None   # Maximum track length kept (None = unlimited)
MN_del_tentative: 10       # Deletions to remove tentative track
MN_confirm_tentative: 15   # Detections to confirm tentative track
MN_del_confirmed: 18       # Deletions to remove confirmed track
MN_intervall: 20           # Total updates considered for M/N logic

# Score logic parameters
lambda: 0.90                    # Forgetting factor (0-1, closer to 1 = slower forgetting)
score_del_confirmed: 0.05       # Score threshold for deletion
```

## 3. Usage & Documentation

For all usage and development documentation related to the `multi_target_tracking` package, please refer to its [own README file](multi_target_tracking/Readme.md) and documentation.

## 4. License

See the [LICENSE file in this repository](LICENSE) and the [LICENSE file in the `multi_target_tracking` submodule](multi_target_tracking/LICENSE) for license information.
