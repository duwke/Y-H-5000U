# Local Development Setup for Arduino IoT Cloud

Complete guide for developing locally with Arduino IoT Cloud OTA deployment.

## Overview

This setup lets you:
- ✅ Edit code in any local IDE (Arduino IDE 2, VSCode, vim, etc.)
- ✅ Compile locally (no web IDE)
- ✅ Deploy via OTA using Arduino Cloud infrastructure
- ✅ Use git for version control
- ✅ Automate deployments with scripts

## Prerequisites

- Linux, macOS, or Windows with bash shell
- Arduino R4 WiFi board
- Arduino IoT Cloud account (paid plan recommended)
- Your device already provisioned in Arduino IoT Cloud

## Installation Steps

### 1. Install Arduino CLI

Arduino CLI is used for local compilation.

**Linux/macOS:**
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
mv bin/arduino-cli ~/bin/
# Make sure ~/bin is in your PATH (add to ~/.bashrc if not already there)
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**Verify Installation:**
```bash
arduino-cli version
```

### 2. Install Arduino Cloud CLI

Arduino Cloud CLI handles OTA deployments and device management.

**Download:**
- Visit: https://github.com/arduino/arduino-cloud-cli/releases
- Download the latest release for your platform
- Extract and move to PATH

**Linux Example:**
```bash
# Download latest release (check GitHub for current version)
wget https://github.com/arduino/arduino-cloud-cli/releases/download/0.4.0/arduino-cloud-cli_0.4.0_Linux_64bit.tar.gz

# Extract
tar -xzf arduino-cloud-cli_0.4.0_Linux_64bit.tar.gz

# Move to your local bin directory
mv arduino-cloud-cli ~/bin/

# Verify
arduino-cloud-cli version
```

### 3. Configure Arduino CLI

Install the Arduino R4 WiFi board core and required libraries.

```bash
# Initialize configuration
arduino-cli config init

# Update board index
arduino-cli core update-index

# Install Arduino R4 WiFi board core
arduino-cli core install arduino:renesas_uno

# Install required libraries
arduino-cli lib install ArduinoIoTCloud
arduino-cli lib install Arduino_ConnectionHandler

# Verify installation
arduino-cli core list
arduino-cli lib list
```

### 4. Configure Arduino Cloud CLI

Get API credentials and authenticate.

**Step 1: Get API Credentials**
1. Go to https://cloud.arduino.cc/home/api-keys
2. Click "CREATE API KEY"
3. Save your Client ID and Client Secret

**Step 2: Initialize Credentials**

**Option A: Interactive Setup**
```bash
arduino-cloud-cli credentials init
# Follow the prompts to enter your Client ID and Secret
```

**Option B: Using Config File**
```bash
# Copy the template
cp credentials.yml.example credentials.yml

# Edit with your credentials
nano credentials.yml
# Fill in client_id and client_secret

# Initialize from file
arduino-cloud-cli credentials init --credentials credentials.yml
```

**Option C: Environment Variables**
```bash
# Add to your ~/.bashrc or ~/.zshrc
export ARDUINO_CLOUD_CLIENT="your-client-id"
export ARDUINO_CLOUD_SECRET="your-client-secret"
```

**Verify Authentication:**
```bash
arduino-cloud-cli device list
```

If you see your devices, authentication is successful!

### 5. Find Your Device ID

You'll need your device ID for deployments.

```bash
# List all devices
arduino-cloud-cli device list

# Example output:
# ID                                    NAME              TYPE    FQBN
# abc123-def456-ghi789-jkl012-mno345   My Device         wifi    arduino:renesas_uno:unor4wifi
```

**Save Your Device ID:**
```bash
# Method 1: Environment variable (add to ~/.bashrc)
export ARDUINO_DEVICE_ID="abc123-def456-ghi789-jkl012-mno345"

# Method 2: Pass directly to deploy scripts
./deploy.sh abc123-def456-ghi789-jkl012-mno345
```

## Usage

### Compile Locally

```bash
# Compile with arduino-cli
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .

# Compile and export binary
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --output-dir ./build .
```

### Deploy via OTA

**Option 1: Using Bash Script** (Recommended)
```bash
# Deploy to specific device
./deploy.sh your-device-id

# Or with environment variable
export ARDUINO_DEVICE_ID=your-device-id
./deploy.sh
```

**Option 2: Using Python Script**
```bash
# Deploy to specific device
python3 deploy.py your-device-id

# Or with environment variable
export ARDUINO_DEVICE_ID=your-device-id
python3 deploy.py
```

**Option 3: Manual Steps**
```bash
# 1. Compile
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --output-dir ./build .

# 2. Upload via cloud
arduino-cloud-cli ota upload --device-id your-device-id --file ./build/YOUR-SKETCH-NAME.ino.bin

# 3. Check status
arduino-cloud-cli ota status --device-id your-device-id
```

### VSCode Integration

If using Visual Studio Code:

1. Open Command Palette: `Ctrl+Shift+P` (Linux/Windows) or `Cmd+Shift+P` (macOS)
2. Type: "Tasks: Run Task"
3. Select from available tasks:
   - **Arduino: Compile** - Compile sketch locally
   - **Arduino Cloud: Deploy OTA** - Deploy to device
   - **Arduino Cloud: List Devices** - Show all devices
   - **Arduino Cloud: Check OTA Status** - Check deployment status
   - **Arduino: Serial Monitor** - Monitor serial output

**Keyboard Shortcut:**
- `Ctrl+Shift+B` (Linux/Windows) or `Cmd+Shift+B` (macOS) - Run default build task (compile)

## Development Workflow

### Day-to-Day Development

1. **Edit Code**
   - Use any editor: Arduino IDE 2, VSCode, vim, nano, etc.
   - Edit your main `.ino` sketch file directly

2. **Test Locally (Optional)**
   ```bash
   # Compile to check for errors
   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .

   # Upload via USB for quick testing
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_uno:unor4wifi .
   ```

3. **Deploy to Production**
   ```bash
   ./deploy.sh your-device-id
   ```

4. **Monitor**
   - Arduino IoT Cloud Dashboard: https://app.arduino.cc/things/YOUR-THING-ID
   - Built-in Web Server: Check deviceIP variable in cloud dashboard for URL
   - Serial Monitor: `arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200`

### Git Workflow

```bash
# Make changes
git add your-sketch.ino

# Commit
git commit -m "Add feature: improved functionality"

# Push to remote
git push origin main

# Deploy to device
./deploy.sh
```

## File Structure

```
project-folder/
├── your-sketch.ino                # Main sketch (edit this)
├── thingProperties.h              # Arduino IoT Cloud configuration
├── arduino_secrets.h              # Your secrets (not in git)
├── arduino_secrets.h.example      # Template for secrets
├── deploy.sh                      # Bash deployment script
├── deploy.py                      # Python deployment script
├── credentials.yml.example        # Cloud CLI credentials template
├── SETUP.md                       # This file
├── README.md                      # Project documentation
├── .gitignore                     # Protects secrets from git
├── .vscode/                       # VSCode integration
│   └── tasks.json                 # VSCode tasks
└── build/                         # Compiled binaries (not in git)
```

## Troubleshooting

### Compilation Errors

**Error: Platform not installed**
```bash
arduino-cli core install arduino:renesas_uno
```

**Error: Library not found**
```bash
arduino-cli lib install ArduinoIoTCloud
arduino-cli lib install Arduino_ConnectionHandler
```

### OTA Upload Errors

**Error: Device not found**
- Check device ID: `arduino-cloud-cli device list`
- Verify device is online in Arduino IoT Cloud dashboard

**Error: Authentication failed**
- Re-initialize credentials: `arduino-cloud-cli credentials init`
- Check API key at: https://cloud.arduino.cc/home/api-keys

**Error: Upload timeout**
- Device may be offline or not connected to WiFi
- Check device connection in cloud dashboard
- Try again in a few minutes

### Network Issues

**Device not connecting to cloud**
1. Check WiFi credentials in Network Configurator
2. Use BLE provisioning: Arduino IoT Cloud app on phone
3. Check Serial Monitor for connection errors:
   ```bash
   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
   ```

## Advanced Usage

### Fleet Deployment

Deploy to multiple devices with tags:

```bash
# Tag devices
arduino-cloud-cli device create-tags --id device-id-1 --tags env=production,location=warehouse-1
arduino-cloud-cli device create-tags --id device-id-2 --tags env=production,location=warehouse-1

# Mass deploy to all tagged devices
arduino-cloud-cli ota mass-upload \
  --fqbn arduino:renesas_uno:unor4wifi \
  --device-tags env=production \
  --file ./build/your-sketch.ino.bin
```

### CI/CD Integration

Example GitHub Actions workflow:

```yaml
name: Deploy to Arduino Cloud

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Install Arduino CLI
        run: |
          curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
          sudo mv bin/arduino-cli /usr/local/bin/

      - name: Setup Arduino
        run: |
          arduino-cli core install arduino:renesas_uno
          arduino-cli lib install ArduinoIoTCloud Arduino_ConnectionHandler

      - name: Compile
        run: arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --output-dir ./build .

      - name: Deploy (if needed)
        run: |
          # Install Arduino Cloud CLI
          # Deploy via OTA using secrets
          # arduino-cloud-cli ota upload --device-id ${{ secrets.DEVICE_ID }} --file ./build/your-sketch.ino.bin
```

### Custom Build Scripts

Create custom deployment workflows:

```bash
# deploy-staging.sh
export ARDUINO_DEVICE_ID="staging-device-id"
./deploy.sh

# deploy-production.sh
export ARDUINO_DEVICE_ID="production-device-id"
./deploy.sh
```

## Resources

- **Arduino CLI Documentation**: https://arduino.github.io/arduino-cli/
- **Arduino Cloud CLI**: https://github.com/arduino/arduino-cloud-cli
- **Arduino IoT Cloud**: https://cloud.arduino.cc/
- **Arduino R4 WiFi**: https://docs.arduino.cc/hardware/uno-r4-wifi/

## Support

**Arduino Community:**
- Forum: https://forum.arduino.cc/
- Discord: https://discord.gg/arduino

**Project Specific:**
- Check built-in web server for runtime logs (see deviceIP in cloud dashboard)
- Review Serial Monitor output for debugging
- Consult Arduino IoT Cloud dashboard for connectivity status

## Next Steps

1. **Test Compilation**
   ```bash
   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .
   ```

2. **Find Your Device ID**
   ```bash
   arduino-cloud-cli device list
   ```

3. **First Deployment**
   ```bash
   ./deploy.sh your-device-id
   ```

4. **Monitor Deployment**
   - Watch terminal output for deployment status
   - Check Arduino IoT Cloud dashboard
   - View logs on built-in web server (deviceIP variable)

**You're ready to develop locally with Arduino IoT Cloud! 🚀**
