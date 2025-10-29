#!/bin/bash

################################################################################
# Arduino IoT Cloud OTA Deployment Script
# For Y&H HSI 5000U Solar Inverter Monitor
################################################################################

# Configuration
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_NAME="Y-H-5000U"
FQBN="arduino:renesas_uno:unor4wifi"
BUILD_DIR="$PROJECT_DIR/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get device ID from command line or environment variable
DEVICE_ID="${1:-$ARDUINO_DEVICE_ID}"

if [ -z "$DEVICE_ID" ]; then
    echo -e "${RED}❌ Error: Device ID not provided${NC}"
    echo "Usage: $0 <device-id>"
    echo "Or set environment variable: export ARDUINO_DEVICE_ID=your-device-id"
    echo ""
    echo "To find your device ID:"
    echo "  arduino-cloud-cli device list"
    exit 1
fi

echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Arduino IoT Cloud OTA Deployment${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "📦 Project: ${GREEN}Y&H HSI 5000U Solar Monitor${NC}"
echo -e "🎯 Device ID: ${GREEN}$DEVICE_ID${NC}"
echo -e "🔧 Board: ${GREEN}Arduino R4 WiFi${NC}"
echo ""

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo -e "${RED}❌ arduino-cli not found${NC}"
    echo "Install it from: https://arduino.github.io/arduino-cli/latest/installation/"
    exit 1
fi

# Check if arduino-cloud-cli is installed
if ! command -v arduino-cloud-cli &> /dev/null; then
    echo -e "${RED}❌ arduino-cloud-cli not found${NC}"
    echo "Download from: https://github.com/arduino/arduino-cloud-cli/releases"
    exit 1
fi

# Step 1: Clean previous builds
echo -e "${YELLOW}[1/5]${NC} Cleaning previous builds..."
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
echo -e "${GREEN}✓${NC} Build directory ready"
echo ""

# Step 2: Compile sketch
echo -e "${YELLOW}[2/5]${NC} Compiling sketch..."
arduino-cli compile \
    --fqbn "$FQBN" \
    --output-dir "$BUILD_DIR" \
    "$PROJECT_DIR"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Compilation failed!${NC}"
    echo "Check the error messages above for details."
    exit 1
fi

echo -e "${GREEN}✓${NC} Compilation successful"
echo ""

# Step 3: Find compiled binary
echo -e "${YELLOW}[3/5]${NC} Locating binary file..."
BINARY_FILE=$(find "$BUILD_DIR" -name "${SKETCH_NAME}.ino.bin" | head -n 1)

if [ -z "$BINARY_FILE" ]; then
    echo -e "${RED}❌ Could not find compiled binary!${NC}"
    echo "Expected: $BUILD_DIR/${SKETCH_NAME}.ino.bin"
    exit 1
fi

BINARY_SIZE=$(du -h "$BINARY_FILE" | cut -f1)
echo -e "${GREEN}✓${NC} Binary found: $BINARY_FILE (${BINARY_SIZE})"
echo ""

# Step 4: Upload via Arduino Cloud OTA
echo -e "${YELLOW}[4/5]${NC} Uploading to Arduino Cloud..."
echo "This may take 30-60 seconds..."

arduino-cloud-cli ota upload \
    --device-id "$DEVICE_ID" \
    --file "$BINARY_FILE"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ OTA upload failed!${NC}"
    echo "Common issues:"
    echo "  - Device not connected to cloud"
    echo "  - Invalid device ID"
    echo "  - Arduino Cloud CLI not authenticated (run: arduino-cloud-cli credentials init)"
    exit 1
fi

echo -e "${GREEN}✓${NC} Upload initiated successfully"
echo ""

# Step 5: Check deployment status
echo -e "${YELLOW}[5/5]${NC} Checking deployment status..."
sleep 3  # Give device time to start update

arduino-cloud-cli ota status --device-id "$DEVICE_ID"

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✅ Deployment Complete!${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
echo ""
echo "📊 Monitor your device at:"
echo "   https://app.arduino.cc/things/$DEVICE_ID"
echo ""
echo "🔍 View webhook logs at:"
echo "   https://webhook.site/825ddff9-6a42-4654-87d5-945624002909"
echo ""
echo "⏱️  Timestamp: $(date)"
