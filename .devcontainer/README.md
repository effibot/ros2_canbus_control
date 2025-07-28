# ROS2 CAN Bus Control - DevContainer Configurations

This repository provides multiple DevContainer configurations optimized for different GPU setups and graphics environments. Each configuration supports both **Xorg** and **Wayland** display servers.

## Available Configurations

### 1. Intel/Integrated GPU Configuration (`.devcontainer/intel-gpu/`)
- **Best for**: Systems with Intel integrated graphics or Intel discrete GPUs
- **Features**: 
  - Hardware acceleration via Intel VA-API
  - Optimized for Intel graphics drivers
  - Lower power consumption
  - Compatible with both Xorg and Wayland

### 2. NVIDIA GPU Configuration (`.devcontainer/nvidia-gpu/`)
- **Best for**: Systems with NVIDIA discrete GPUs
- **Features**: 
  - CUDA support for GPU-accelerated computing
  - NVIDIA runtime for container GPU access
  - OpenGL hardware acceleration
  - Compatible with both Xorg and Wayland
  - Hybrid GPU support (Intel + NVIDIA)

## Prerequisites

### Required Software
1. **Docker**: [Install Docker Engine](https://docs.docker.com/engine/install/)
2. **VSCode**: [Download VSCode](https://code.visualstudio.com/)
3. **Dev Containers Extension**: Install from VSCode marketplace
4. **Docker Compose**: Usually included with Docker Desktop

### GPU-Specific Prerequisites

#### For NVIDIA GPU Support
```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Verify installation
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

#### For Intel GPU Support
```bash
# Install Intel GPU tools (optional, for debugging)
sudo apt-get update
sudo apt-get install intel-gpu-tools vainfo
```

## Usage Instructions

### Method 1: Using the Setup Script (Recommended)
1. Run the interactive setup script:
   ```bash
   ./.devcontainer/setup-devcontainer.sh
   ```
2. Select your GPU configuration
3. Follow the provided instructions to navigate to the appropriate folder
4. Open VSCode from that folder

### Method 2: Direct Navigation
1. Navigate directly to your preferred configuration folder:
   ```bash
   # For Intel GPU
   cd .devcontainer/intel-gpu
   code .
   
   # For NVIDIA GPU
   cd .devcontainer/nvidia-gpu
   code .
   ```
2. VSCode will automatically detect the devcontainer configuration
3. Click "Reopen in Container" when prompted

### Method 3: VSCode Command Palette
1. Open VSCode in the project root
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "Dev Containers: Open Folder in Container"
4. Navigate to and select either:
   - `.devcontainer/intel-gpu/` for Intel GPU configuration
   - `.devcontainer/nvidia-gpu/` for NVIDIA GPU configuration

### Method 4: Quick Command Line Options
```bash
# Show Intel GPU instructions
./.devcontainer/setup-devcontainer.sh intel

# Show NVIDIA GPU instructions
./.devcontainer/setup-devcontainer.sh nvidia

# Directly open Intel GPU config in VSCode
./.devcontainer/setup-devcontainer.sh open-intel

# Directly open NVIDIA GPU config in VSCode
./.devcontainer/setup-devcontainer.sh open-nvidia
```

## Graphics Support Details

### Xorg Support
Both configurations automatically support Xorg by:
- Mounting `/tmp/.X11-unix` for X11 socket communication
- Setting `DISPLAY` environment variable
- Providing `QT_QPA_PLATFORM=xcb` for Qt applications

### Wayland Support
Both configurations support Wayland by:
- Mounting Wayland display socket via `XDG_RUNTIME_DIR`
- Setting `WAYLAND_DISPLAY` environment variable
- Auto-detecting session type via `XDG_SESSION_TYPE`

### Audio Support
PulseAudio support is included for multimedia applications:
- Mounts PulseAudio socket from host
- Sets `PULSE_SERVER` environment variable

## Environment Variables

### Common Variables (Both Configurations)
- `DISPLAY`: X11 display identifier
- `WAYLAND_DISPLAY`: Wayland display identifier  
- `XDG_RUNTIME_DIR`: Runtime directory for user session
- `XDG_SESSION_TYPE`: Session type (x11 or wayland)
- `QT_QPA_PLATFORM`: Qt platform abstraction
- `ROS_DOMAIN_ID`: ROS2 domain identifier (set to 42)
- `ROS_AUTOMATIC_DISCOVERY_RANGE`: ROS2 discovery scope

### Intel GPU Specific
- `LIBVA_DRIVER_NAME=iHD`: Intel VA-API driver
- `INTEL_MEDIA_RUNTIME=ONEVPL`: Intel media runtime
- `GST_VAAPI_ALL_DRIVERS=1`: GStreamer VA-API support

### NVIDIA GPU Specific
- `NVIDIA_VISIBLE_DEVICES=all`: All NVIDIA devices visible
- `NVIDIA_DRIVER_CAPABILITIES=all`: All NVIDIA capabilities
- `__GLX_VENDOR_LIBRARY_NAME=nvidia`: NVIDIA OpenGL
- `__NV_PRIME_RENDER_OFFLOAD=1`: NVIDIA PRIME offloading

## Testing Graphics Setup

### Test X11 Applications
```bash
# Install and test xeyes (if not available)
sudo apt-get install x11-apps
xeyes

# Test with a simple GUI
xclock
```

### Test OpenGL Acceleration
```bash
# Install mesa utilities
sudo apt-get install mesa-utils

# Test OpenGL
glxinfo | grep "OpenGL renderer"
glxgears
```

### Test ROS2 GUI Applications
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Test RViz2
rviz2

# Test rqt
rqt
```

## Troubleshooting

### Graphics Not Working
1. **Check display variables**:
   ```bash
   echo $DISPLAY
   echo $WAYLAND_DISPLAY
   echo $XDG_SESSION_TYPE
   ```

2. **For Wayland users**:
   ```bash
   # You might need to set XDG_RUNTIME_DIR
   export XDG_RUNTIME_DIR=/run/user/$(id -u)
   ```

3. **For X11 users**:
   ```bash
   # Check X11 authorization
   xauth list
   # If empty, you might need to run:
   xhost +local:docker
   ```

### NVIDIA GPU Issues
1. **Verify NVIDIA container runtime**:
   ```bash
   docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

2. **Check NVIDIA driver**:
   ```bash
   nvidia-smi
   ```

### Intel GPU Issues
1. **Check DRI devices**:
   ```bash
   ls -la /dev/dri/
   ```

2. **Verify VA-API**:
   ```bash
   vainfo
   ```

### Permission Issues
```bash
# Fix USB device permissions (if needed)
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# Fix graphics device permissions
sudo usermod -a -G video $USER
sudo usermod -a -G render $USER
```

## Customization

### Adding Extensions
Edit the `extensions` array in the respective `devcontainer.json`:
```json
"extensions": [
    "existing.extension",
    "your.new.extension"
]
```

### Modifying Environment Variables
Edit the `environment` section in the respective `compose.yaml`:
```yaml
environment:
  - YOUR_VARIABLE=value
```

### Adding Volume Mounts
Edit the `volumes` section in the respective `compose.yaml`:
```yaml
volumes:
  - type: bind
    source: /host/path
    target: /container/path
```

## Performance Tips

### Intel GPU Configuration
- Use for lightweight GUI applications
- Better battery life on laptops
- Sufficient for most ROS2 visualization tasks

### NVIDIA GPU Configuration  
- Use for CUDA-accelerated computing
- Better for machine learning workloads
- Required for GPU-accelerated simulation
- Higher power consumption

### Hybrid Systems
If you have both Intel and NVIDIA GPUs, you can:
1. Use Intel configuration for general development
2. Switch to NVIDIA configuration for GPU-intensive tasks
3. The NVIDIA configuration includes Intel GPU support for flexibility

## Support

For issues specific to:
- **DevContainer setup**: Check VSCode Dev Containers documentation
- **Docker**: Check Docker documentation  
- **GPU drivers**: Check your GPU manufacturer's documentation
- **ROS2**: Check ROS2 documentation

Remember to restart your system after installing GPU drivers or container runtimes.
