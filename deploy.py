#!/usr/bin/env python3
"""
Arduino IoT Cloud OTA Deployment Script (Python)
For Y&H HSI 5000U Solar Inverter Monitor
"""

import subprocess
import sys
import os
from pathlib import Path
import time
from datetime import datetime

# Configuration
PROJECT_DIR = Path(__file__).parent
SKETCH_NAME = "monitor"
FQBN = "arduino:renesas_uno:unor4wifi"
BUILD_DIR = PROJECT_DIR / "build"

# ANSI Colors
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    BOLD = '\033[1m'
    NC = '\033[0m'  # No Color


def print_header():
    """Print deployment header"""
    print(f"{Colors.BLUE}{'═' * 59}{Colors.NC}")
    print(f"{Colors.BLUE}  Arduino IoT Cloud OTA Deployment{Colors.NC}")
    print(f"{Colors.BLUE}{'═' * 59}{Colors.NC}\n")


def print_step(step_num, total_steps, message):
    """Print step information"""
    print(f"{Colors.YELLOW}[{step_num}/{total_steps}]{Colors.NC} {message}...")


def print_success(message):
    """Print success message"""
    print(f"{Colors.GREEN}✓{Colors.NC} {message}")


def print_error(message):
    """Print error message"""
    print(f"{Colors.RED}❌ {message}{Colors.NC}")


def run_command(cmd, description, capture_output=False):
    """
    Run a shell command and handle errors

    Args:
        cmd: Command to run (list or string)
        description: Description for error messages
        capture_output: Whether to capture output

    Returns:
        CompletedProcess object if capture_output=True, else bool
    """
    try:
        if isinstance(cmd, str):
            cmd = cmd.split()

        if capture_output:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=True
            )
            return result
        else:
            result = subprocess.run(cmd, check=True)
            return True

    except subprocess.CalledProcessError as e:
        print_error(f"{description} failed!")
        if capture_output and e.stderr:
            print(e.stderr)
        return False
    except FileNotFoundError:
        print_error(f"Command not found: {cmd[0]}")
        return False


def check_tools():
    """Check if required tools are installed"""
    tools = {
        'arduino-cli': 'https://arduino.github.io/arduino-cli/latest/installation/',
        'arduino-cloud-cli': 'https://github.com/arduino/arduino-cloud-cli/releases'
    }

    for tool, install_url in tools.items():
        if subprocess.run(['which', tool], capture_output=True).returncode != 0:
            print_error(f"{tool} not found")
            print(f"Install it from: {install_url}")
            return False

    return True


def clean_build_dir():
    """Clean and recreate build directory"""
    if BUILD_DIR.exists():
        import shutil
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir(parents=True)


def compile_sketch():
    """Compile the Arduino sketch"""
    cmd = [
        'arduino-cli', 'compile',
        '--fqbn', FQBN,
        '--output-dir', str(BUILD_DIR),
        str(PROJECT_DIR)
    ]

    return run_command(cmd, "Compilation")


def find_binary():
    """Find the compiled binary file"""
    binary_pattern = f"{SKETCH_NAME}.ino.bin"
    binaries = list(BUILD_DIR.rglob(binary_pattern))

    if not binaries:
        print_error(f"Could not find compiled binary: {binary_pattern}")
        return None

    binary_file = binaries[0]
    size_mb = binary_file.stat().st_size / (1024 * 1024)
    print_success(f"Binary found: {binary_file.name} ({size_mb:.2f} MB)")
    return binary_file


def upload_ota(device_id, binary_file):
    """Upload binary via Arduino Cloud OTA"""
    cmd = [
        'arduino-cloud-cli', 'ota', 'upload',
        '--device-id', device_id,
        '--file', str(binary_file)
    ]

    print("This may take 30-60 seconds...")
    return run_command(cmd, "OTA upload")


def check_status(device_id):
    """Check OTA deployment status"""
    time.sleep(3)  # Give device time to start update

    cmd = [
        'arduino-cloud-cli', 'ota', 'status',
        '--device-id', device_id
    ]

    result = run_command(cmd, "Status check", capture_output=True)
    if result:
        print(result.stdout)


def print_footer(device_id):
    """Print deployment completion footer"""
    print(f"\n{Colors.BLUE}{'═' * 59}{Colors.NC}")
    print(f"{Colors.GREEN}✅ Deployment Complete!{Colors.NC}")
    print(f"{Colors.BLUE}{'═' * 59}{Colors.NC}\n")

    print("📊 Monitor your device at:")
    print(f"   https://app.arduino.cc/things/{device_id}\n")

    print("🔍 View webhook logs at:")
    print("   https://webhook.site/825ddff9-6a42-4654-87d5-945624002909\n")

    print(f"⏱️  Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


def main():
    """Main deployment workflow"""
    # Get device ID from arguments or environment
    device_id = None
    if len(sys.argv) > 1:
        device_id = sys.argv[1]
    else:
        device_id = os.environ.get('ARDUINO_DEVICE_ID')

    if not device_id:
        print_error("Device ID not provided")
        print("Usage: python3 deploy.py <device-id>")
        print("Or set environment variable: export ARDUINO_DEVICE_ID=your-device-id\n")
        print("To find your device ID:")
        print("  arduino-cloud-cli device list")
        sys.exit(1)

    # Print header
    print_header()
    print(f"📦 Project: {Colors.GREEN}Y&H HSI 5000U Solar Monitor{Colors.NC}")
    print(f"🎯 Device ID: {Colors.GREEN}{device_id}{Colors.NC}")
    print(f"🔧 Board: {Colors.GREEN}Arduino R4 WiFi{Colors.NC}\n")

    # Check required tools
    if not check_tools():
        sys.exit(1)

    # Step 1: Clean build directory
    print_step(1, 5, "Cleaning previous builds")
    clean_build_dir()
    print_success("Build directory ready\n")

    # Step 2: Compile sketch
    print_step(2, 5, "Compiling sketch")
    if not compile_sketch():
        print("\nCommon compilation issues:")
        print("  - Missing libraries (run: arduino-cli lib install ArduinoIoTCloud)")
        print("  - Missing board core (run: arduino-cli core install arduino:renesas_uno)")
        sys.exit(1)
    print_success("Compilation successful\n")

    # Step 3: Find binary
    print_step(3, 5, "Locating binary file")
    binary_file = find_binary()
    if not binary_file:
        sys.exit(1)
    print()

    # Step 4: Upload via OTA
    print_step(4, 5, "Uploading to Arduino Cloud")
    if not upload_ota(device_id, binary_file):
        print("\nCommon OTA issues:")
        print("  - Device not connected to cloud")
        print("  - Invalid device ID")
        print("  - Arduino Cloud CLI not authenticated")
        print("    (run: arduino-cloud-cli credentials init)")
        sys.exit(1)
    print_success("Upload initiated successfully\n")

    # Step 5: Check status
    print_step(5, 5, "Checking deployment status")
    check_status(device_id)

    # Print footer
    print_footer(device_id)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}⚠️  Deployment interrupted by user{Colors.NC}")
        sys.exit(130)
    except Exception as e:
        print_error(f"Unexpected error: {e}")
        sys.exit(1)
