# ONNX Runtime Clean Installation Guide

A comprehensive guide for completely removing and reinstalling ONNX Runtime with GPU support on ARM64 systems.

## Prerequisites

- Python 3.10
- CUDA toolkit installed
- ARM64 Linux system
- Administrative privileges

## Complete Clean Installation Process

### Step 1: Remove All Existing ONNX Packages

Remove all ONNX-related packages from your system:

```bash
# Uninstall all ONNX packages (run twice to ensure complete removal)
pip uninstall onnxruntime onnxruntime-gpu onnx onnxoptimizer -y
pip uninstall onnxruntime onnxruntime-gpu onnx onnxoptimizer -y
```

### Step 2: Manual File System Cleanup

Remove any remaining installation files:

```bash
# Remove local installation directories
rm -rf ~/.local/lib/python3.10/site-packages/onnxruntime*
rm -rf ~/.local/lib/python3.10/site-packages/onnx*

# Find and remove any remaining ONNX-related directories
find ~/.local/lib/python3.10/site-packages -name "*onnx*" -type d -exec rm -rf {} + 2>/dev/null || true
```

### Step 3: Clear Package Caches

Clean all pip and Python caches:

```bash
# Clear pip cache
pip cache purge

# Clear Python import cache
python3 -c "import sys; print('Python cache cleared')"
```

### Step 4: Verify Complete Removal

Confirm that ONNX Runtime has been completely removed:

```bash
python3 -c "
try:
    import onnxruntime
    print('ERROR: ONNX Runtime still found - cleanup incomplete')
    exit(1)
except ImportError:
    print('SUCCESS: ONNX Runtime completely removed')
"
```

### Step 5: Install Fresh Dependencies

Install required dependencies with specific versions:

```bash
# Upgrade pip
pip install --upgrade pip

# Install dependencies with compatible versions
pip install numpy==1.23.5
pip install protobuf==4.21.12
pip install flatbuffers
pip install sympy
pip install packaging
```

### Step 6: Download ONNX Runtime Wheel

Download the verified ARM64 wheel file:

```bash
# Set variables
WHEEL_URL="https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl"
WHEEL_FILE="onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl"

# Remove any existing wheel file
rm -f $WHEEL_FILE

# Download the wheel
wget --progress=bar:force:noscroll -O $WHEEL_FILE $WHEEL_URL

# Verify download
if [ -f "$WHEEL_FILE" ]; then
    echo "Wheel downloaded successfully"
    ls -lh $WHEEL_FILE
else
    echo "Download failed"
    exit 1
fi
```

### Step 7: Install ONNX Runtime

Install the downloaded wheel with forced reinstallation:

```bash
pip install $WHEEL_FILE --force-reinstall --no-deps --no-cache-dir
```

### Step 8: Cleanup
Remove the downloaded wheel file after successful installation:

```bash
bashrm -f $WHEEL_FILE
```