#!/bin/bash
# ============================================================================
# Jetson Nano One-Time Setup Script
# Run this ONCE on a fresh Jetson Nano before using the VIO pipeline
# Usage: chmod +x setup_jetson.sh && sudo ./setup_jetson.sh
# ============================================================================

set -e
echo "============================================"
echo "  Jetson Nano Setup for VIO Pipeline"
echo "============================================"

# ── Check if running as root ─────────────────────────────────────────────────
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run as root (sudo ./setup_jetson.sh)"
    exit 1
fi

CURRENT_USER=${SUDO_USER:-$USER}

# ── Step 1: Add swap space (8GB) ─────────────────────────────────────────────
echo ""
echo "[1/6] Setting up 8GB swap file..."
if [ ! -f /var/swapfile ]; then
    # Disable ZRAM first
    systemctl disable nvzramconfig 2>/dev/null || true
    systemctl stop nvzramconfig 2>/dev/null || true

    # Create swap file
    fallocate -l 8G /var/swapfile
    chmod 600 /var/swapfile
    mkswap /var/swapfile
    swapon /var/swapfile

    # Make persistent
    if ! grep -q "swapfile" /etc/fstab; then
        echo "/var/swapfile none swap sw 0 0" >> /etc/fstab
    fi
    echo "  ✓ 8GB swap file created and activated"
else
    echo "  ✓ Swap file already exists"
fi

# ── Step 2: Install Docker ───────────────────────────────────────────────────
echo ""
echo "[2/6] Checking Docker installation..."
if ! command -v docker &> /dev/null; then
    echo "  Installing Docker..."
    curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
    sh /tmp/get-docker.sh
    rm /tmp/get-docker.sh
    echo "  ✓ Docker installed"
else
    echo "  ✓ Docker already installed"
fi

# ── Step 3: Set up NVIDIA Container Toolkit ──────────────────────────────────
echo ""
echo "[3/6] Setting up NVIDIA Container Toolkit..."
if ! dpkg -l | grep -q nvidia-container-toolkit 2>/dev/null; then
    # For JetPack 4.x, nvidia-docker2 should already be available
    apt-get update
    apt-get install -y nvidia-docker2 || apt-get install -y nvidia-container-toolkit || true
    systemctl restart docker
    echo "  ✓ NVIDIA Container Toolkit installed"
else
    echo "  ✓ NVIDIA Container Toolkit already installed"
fi

# Set nvidia as default runtime
if [ -f /etc/docker/daemon.json ]; then
    python3 -c "
import json
with open('/etc/docker/daemon.json', 'r') as f:
    config = json.load(f)
config['default-runtime'] = 'nvidia'
with open('/etc/docker/daemon.json', 'w') as f:
    json.dump(config, f, indent=2)
" 2>/dev/null || true
fi
systemctl restart docker 2>/dev/null || true

# ── Step 4: Add user to groups ───────────────────────────────────────────────
echo ""
echo "[4/6] Adding user '${CURRENT_USER}' to required groups..."
usermod -aG docker ${CURRENT_USER}
usermod -aG dialout ${CURRENT_USER}
echo "  ✓ Added to 'docker' group (run containers without sudo)"
echo "  ✓ Added to 'dialout' group (access serial devices)"

# ── Step 5: Set up udev rules for RealSense and Pixhawk ─────────────────────
echo ""
echo "[5/6] Setting up udev rules..."

# RealSense udev rules
cat > /etc/udev/rules.d/99-realsense-libusb.rules << 'EOF'
# Intel RealSense — allow access without root
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", MODE:="0666"
EOF

# Pixhawk udev rules (USB serial)
cat > /etc/udev/rules.d/99-pixhawk.rules << 'EOF'
# Pixhawk flight controller — consistent device name
SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", SYMLINK+="pixhawk", MODE:="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", SYMLINK+="pixhawk", MODE:="0666"
# Holybro Pixhawk 6C Mini
SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", SYMLINK+="pixhawk", MODE:="0666"
EOF

udevadm control --reload-rules
udevadm trigger
echo "  ✓ Udev rules created for RealSense and Pixhawk"

# ── Step 6: Disable serial console (free /dev/ttyTHS1 if needed later) ───────
echo ""
echo "[6/6] Disabling serial console on ttyTHS1..."
systemctl stop nvgetty 2>/dev/null || true
systemctl disable nvgetty 2>/dev/null || true
echo "  ✓ Serial console disabled"

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "IMPORTANT: You must log out and log back in"
echo "(or reboot) for group changes to take effect."
echo ""
echo "Next steps:"
echo "  1. Reboot:  sudo reboot"
echo "  2. Verify:  docker run --rm hello-world"
echo "  3. Build:   cd openvins_vio && ./scripts/build_docker.sh"
echo ""
