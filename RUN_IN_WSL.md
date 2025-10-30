## WSL Kernel Compilation for CAN Support

If you're running this project under WSL (Windows Subsystem for Linux), you'll need to recompile the WSL kernel to enable CAN bus support, as the default WSL kernel doesn't include CAN modules.

### Prerequisites

```bash
# Install required build tools
sudo apt update
sudo apt install -y build-essential flex bison libssl-dev libelf-dev bc
```

### Step 1: Get the WSL Kernel Source

```bash
# Create a directory for kernel compilation
mkdir -p ~/wsl-kernel
cd ~/wsl-kernel

# Clone the WSL kernel source (use the version matching your WSL)
git clone https://github.com/microsoft/WSL2-Linux-Kernel.git
cd WSL2-Linux-Kernel

# Check your current kernel version
KERNEL_VERSION=$(uname -r)
echo "Current kernel version: $KERNEL_VERSION"

# Extract the major.minor version for branch matching
MAJOR_MINOR=$(echo $KERNEL_VERSION | sed 's/\([0-9]*\.[0-9]*\).*/\1/')
echo "Looking for branch: linux-msft-wsl-$MAJOR_MINOR.y"

# List available branches and find the matching one
git branch -r | grep "linux-msft-wsl-$MAJOR_MINOR"

# Automatically checkout the matching branch
if git show-ref --verify --quiet refs/remotes/origin/linux-msft-wsl-$MAJOR_MINOR.y; then
    echo "Checking out linux-msft-wsl-$MAJOR_MINOR.y"
    git checkout linux-msft-wsl-$MAJOR_MINOR.y
else
    echo "Branch linux-msft-wsl-$MAJOR_MINOR.y not found. Available branches:"
    git branch -r | grep linux-msft-wsl
    echo "Please manually checkout the appropriate branch"
fi
```

// ...existing code...

### Step 4: Compile the Kernel

```bash
# Compile the kernel (this will take 15-30 minutes)
make -j$(nproc) KCONFIG_CONFIG=.config

# The compiled kernel will be at vmlinux (uncompressed) or arch/x86/boot/bzImage (compressed)
# For WSL, we can use either, but vmlinux is more common
ls -la vmlinux arch/x86/boot/bzImage
```

### Step 5: Install the Custom Kernel

```bash
# Copy the kernel to Windows filesystem
# Use vmlinux (uncompressed kernel image)
cp vmlinux /mnt/c/Users/[YOUR_USERNAME]/

# Alternative: if you prefer the compressed version
# cp arch/x86/boot/bzImage /mnt/c/Users/[YOUR_USERNAME]/
```

### Step 6: Configure WSL to Use Custom Kernel

Create or edit the WSL configuration file on Windows:

1. Open PowerShell as Administrator
2. Create/edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
# Use vmlinux (uncompressed kernel)
kernel=C:\\Users\\[YOUR_USERNAME]\\vmlinux

# Alternative: if using compressed kernel
# kernel=C:\\Users\\[YOUR_USERNAME]\\bzImage
```

### Step 2: Configure the Kernel

```bash
# Copy the existing config
cp Microsoft/config-wsl .config

# Edit the configuration to enable CAN support
make menuconfig
```

### Step 3: Required Kernel Configuration Changes

In the `menuconfig` interface, navigate and enable the following options:

```
Networking support --->
    CAN bus subsystem support --->
        [M] CAN bus subsystem support
            CAN Device Drivers --->
                [M] Virtual Local CAN Interface (vcan)
                [M] Serial / USB serial CAN Adaptors (slcan)
            CAN Raw Protocol (raw access with CAN-ID filtering) --->
                [M] CAN Raw Protocol
            CAN Broadcast Manager Protocol (with content filtering) --->
                [M] CAN Broadcast Manager Protocol
```

**Alternative: Direct .config editing**

If you prefer to edit the configuration file directly:

```bash
# Enable CAN support in .config
sed -i 's/# CONFIG_CAN is not set/CONFIG_CAN=m/' .config
echo "CONFIG_CAN_RAW=m" >> .config
echo "CONFIG_CAN_BCM=m" >> .config
echo "CONFIG_CAN_VCAN=m" >> .config
echo "CONFIG_CAN_SLCAN=m" >> .config
echo "CONFIG_CAN_DEV=m" >> .config

# Verify the changes
grep -E "CONFIG_CAN|CONFIG_VCAN" .config
```

Expected output:
```
CONFIG_CAN=m
CONFIG_CAN_RAW=m
CONFIG_CAN_BCM=m
CONFIG_CAN_VCAN=m
CONFIG_CAN_SLCAN=m
CONFIG_CAN_DEV=m
```

### Step 4: Compile the Kernel

```bash
# Compile the kernel (this will take 15-30 minutes)
make -j$(nproc) KCONFIG_CONFIG=.config

# The compiled kernel will be at arch/x86/boot/bzImage
ls -la vmlinux arch/x86/boot/bzImage
```

### Step 5: Install the Custom Kernel

```bash
# Copy the kernel to Windows filesystem
# Use vmlinux (uncompressed kernel image)
cp vmlinux /mnt/c/Users/[YOUR_USERNAME]/

# Alternative: if you prefer the compressed version
# cp arch/x86/boot/bzImage /mnt/c/Users/[YOUR_USERNAME]/
```

### Step 6: Configure WSL to Use Custom Kernel

Create or edit the WSL configuration file on Windows:

1. Open PowerShell as Administrator
2. Create/edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
# Use vmlinux (uncompressed kernel)
kernel=C:\\Users\\[YOUR_USERNAME]\\vmlinux

# Alternative: if using compressed kernel
# kernel=C:\\Users\\[YOUR_USERNAME]\\bzImage
```
### Step 7: Restart WSL

```powershell
# In PowerShell (as Administrator)
wsl --shutdown
wsl
```

### Step 8: Verify CAN Support

After WSL restarts, verify that CAN support is available:

```bash
# Check if CAN modules are available
find /lib/modules/$(uname -r) -name "*can*" -type f

# Load CAN modules
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe vcan

# Verify modules are loaded
lsmod | grep can

# Test virtual CAN interface creation
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip link show vcan0
```

### Troubleshooting

**If compilation fails:**
- Ensure you have enough disk space (at least 10GB free)
- Check that all build dependencies are installed
- Try using fewer parallel jobs: `make -j2` instead of `make -j$(nproc)`

**If WSL doesn't boot with custom kernel:**
- Remove the `kernel=` line from `.wslconfig`
- Restart WSL: `wsl --shutdown && wsl`
- Check kernel compilation for errors

**If CAN modules still not available:**
- Verify the kernel configuration: `zcat /proc/config.gz | grep CAN`
- Ensure modules were compiled as modules (=m) not built-in (=y)
- Check kernel version matches: `uname -r`

---

## Quick Test After Kernel Installation

Once you have CAN support enabled, test the full pipeline:

```bash
# Load modules and create interface
sudo modprobe can can-raw can-bcm vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Test CAN utilities
candump vcan0 &
cansend vcan0 123#DEADBEEF

# You should see the message in candump output
# Kill candump: pkill candump
```

Your WSL environment is now ready for CAN bus development!