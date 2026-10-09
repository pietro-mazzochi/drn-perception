# drn-perception

## Installation

To ensure all necessary dependencies are installed, follow the instructions below.

### System Dependencies

- pandas==2.2.2
- numpy==1.23
- scikit-learn==1.5.1
- open3d==0.18.0
- transforms3d==0.4.2

## Dock Detection Experiments

The launch file `experiment.launch.py` executes dock detection experiments using the nodes from the `drn_perception_reference_detection` package. You can customize the experiment's behavior by adjusting the available parameters.

### **How to Run**
To execute with default values:
```bash
ros2 launch drn_perception_reference_detection experiment.launch.py
```

### **Available Parameters**
| Name          | Type      | Default Value  | Description                                      |
|---------------|-----------|----------------|--------------------------------------------------|
| `base_frame`  | String    | `"base_link"`  | The base frame name used in the coordinate system. |
| `debug`       | Boolean   | `false`        | Enables or disables the debug mode for additional information. If `true`, publishes point cloud used for detection.|
| `direction`   | String    | `"front"`    | Detection direction (`front` or `rear`).   |

### **Notes**
- **Parameter `base_frame`:** Ensure that the specified base frame is valid and consistent with the robot's coordinate system.
- Experiment will publish two topics:
    - `/drn_perception/dock_detector/service_response_topic`: Contains the result of the dock detection.
    - `/drn_perception/dock_detector/debug`: Contains the point cloud data that was used in the detection process.

## Reference Detection Parameters

To simplify the execution of the `update_detection_params.py` script from anywhere, we recommend setting up an alias in your shell configuration file.

```bash
alias perception_update_params="python3 /absolute/path/to/drn-perception/drn_perception_reference_detection/python/update_detection_params.py"
```

Now you can execute the script using the command:

```bash
perception_update_params <arguments>
```

The script `perception_update_params.py` accepts the following arguments:

- `padrao_v_g`
- `padrao_v_m`
- `padrao_v_p`
- `padrao_r_g`
- `padrao_r_m`
- `padrao_r_p`