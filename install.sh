#!/bin/bash
# Husky Robot Control Server Installation Script
# This script installs all dependencies and sets up the ROS2 node as a systemd service

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SERVICE_NAME="husky-rcs"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
WORKING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_NAME="$(whoami)"

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root for system installations
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "This script should not be run as root. It will use sudo when needed."
        exit 1
    fi
}

# Check Ubuntu version
check_ubuntu_version() {
    log_info "Checking Ubuntu version..."
    if [[ ! -f /etc/os-release ]]; then
        log_error "Cannot determine OS version"
        exit 1
    fi

    . /etc/os-release
    if [[ "$ID" != "ubuntu" ]]; then
        log_error "This script is designed for Ubuntu. Current OS: $PRETTY_NAME"
        exit 1
    fi

    if [[ "$VERSION_ID" != "24.04" ]]; then
        log_warning "This script is tested on Ubuntu 24.04. Current version: $VERSION_ID"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi

    log_success "Ubuntu $VERSION_ID detected"
}

# Check and install Python 3.12+
check_python() {
    log_info "Checking Python version..."
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 is not installed"
        exit 1
    fi

    PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
    PYTHON_MAJOR=$(python3 -c 'import sys; print(sys.version_info.major)')
    PYTHON_MINOR=$(python3 -c 'import sys; print(sys.version_info.minor)')

    if [[ $PYTHON_MAJOR -lt 3 ]] || [[ $PYTHON_MAJOR -eq 3 && $PYTHON_MINOR -lt 12 ]]; then
        log_error "Python 3.12+ is required. Current version: $PYTHON_VERSION"
        exit 1
    fi

    log_success "Python $PYTHON_VERSION is installed"
}

# Check and install ROS2 Jazzy
check_ros2() {
    log_info "Checking ROS2 installation..."
    source /opt/ros/jazzy/setup.bash
    if ! command -v ros2 &> /dev/null; then
        log_warning "ROS2 is not installed. Installing ROS2 Jazzy..."

        # Add ROS2 repository
        sudo apt update
        sudo apt install -y software-properties-common curl gnupg lsb-release

        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

        sudo apt update
        sudo apt install -y ros-jazzy-desktop-full

        log_success "ROS2 Jazzy installed successfully"
    else
        ROS2_VERSION=$(ros2 --version | grep -oP 'jazzy|rolling|humble|galactic|foxy' | head -1)
        if [[ "$ROS2_VERSION" != "jazzy" ]]; then
            log_warning "ROS2 version is $ROS2_VERSION, but Jazzy is recommended"
        else
            log_success "ROS2 Jazzy is already installed"
        fi
    fi

    # Install ROS2 Python packages
    log_info "Installing ROS2 Python packages..."
    sudo apt install -y python3-websockets python3-rosdep python3-colcon-common-extensions

    # Initialize rosdep if not already done
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
        rosdep update
    fi
}

# Install uv package manager
install_uv() {
    log_info "Checking uv package manager..."

    if ! command -v uv &> /dev/null; then
        log_info "Installing uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.cargo/bin:$PATH"
        log_success "uv installed successfully"
    else
        log_success "uv is already installed"
    fi
}

# Install Python dependencies
install_python_deps() {
    log_info "Installing Python dependencies..."

    if [[ -f "pyproject.toml" ]]; then
        uv sync
        log_success "Python dependencies installed via uv"
    else
        log_warning "pyproject.toml not found, installing basic dependencies..."
        uv pip install websockets rclpy geometry-msgs
        log_success "Basic Python dependencies installed"
    fi
}

# Create systemd service file
create_service_file() {
    log_info "Creating systemd service file..."

    # Get the absolute path to the working directory
    WORKING_DIR_ABS=$(realpath "$WORKING_DIR")

    # Create service file content
    cat > /tmp/${SERVICE_NAME}.service << EOF
[Unit]
Description=Husky Robot Control Server
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$WORKING_DIR_ABS
Environment=ROS_DOMAIN_ID=0
Environment=PYTHONPATH=$WORKING_DIR_ABS
ExecStartPre=/bin/bash -c 'source /opt/ros/jazzy/setup.bash'
ExecStart=$WORKING_DIR_ABS/.venv/bin/python3 $WORKING_DIR_ABS/husky_rcs.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    # Move service file to system location
    sudo mv /tmp/${SERVICE_NAME}.service $SERVICE_FILE
    sudo chmod 644 $SERVICE_FILE

    log_success "Systemd service file created at $SERVICE_FILE"
}

# Install and enable systemd service
install_service() {
    log_info "Installing systemd service..."

    sudo systemctl daemon-reload
    sudo systemctl enable $SERVICE_NAME

    log_success "Service $SERVICE_NAME enabled"
    log_info "To start the service: sudo systemctl start $SERVICE_NAME"
    log_info "To check status: sudo systemctl status $SERVICE_NAME"
    log_info "To view logs: journalctl -u $SERVICE_NAME -f"
}

# Main installation function
main() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE} Husky Control Server Installer ${NC}"
    echo -e "${BLUE}================================${NC}"
    echo

    check_root
    check_ubuntu_version
    check_python
    install_uv
    install_python_deps
    create_service_file
    install_service

    echo
    echo -e "${GREEN}================================${NC}"
    echo -e "${GREEN} Installation completed! ${NC}"
    echo -e "${GREEN}================================${NC}"
    echo
    log_success "Husky Robot Control Server is ready to use"
    echo
    echo "Next steps:"
    echo "1. Configure environment variables in .env file (optional)"
    echo "2. Start the service: sudo systemctl start $SERVICE_NAME"
    echo "3. Check status: sudo systemctl status $SERVICE_NAME"
    echo "4. View logs: journalctl -u $SERVICE_NAME -f"
    echo
    echo "For manual testing:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ./start.sh"
}

# Run main function
main "$@"