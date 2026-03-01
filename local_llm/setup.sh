#!/bin/bash
# =============================================================================
# Wolfwagen Local Voice Assistant - Setup Script for Jetson Orin
# =============================================================================
# This script installs all dependencies:
#   1. Ollama (local LLM server)
#   2. Piper TTS (text-to-speech)
#   3. Python dependencies (faster-whisper, silero-vad, pyaudio, etc.)
#   4. Downloads default models
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPER_DIR="$SCRIPT_DIR/piper"
PIPER_MODELS_DIR="$SCRIPT_DIR/piper_models"

echo "=========================================="
echo " Wolfwagen Local Voice Assistant Setup"
echo "=========================================="

# --- System dependencies ---
echo ""
echo "[1/5] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    libsndfile1 \
    ffmpeg \
    curl \
    wget

# --- Ollama ---
echo ""
echo "[2/5] Installing Ollama..."
if command -v ollama &> /dev/null; then
    echo "  Ollama already installed: $(ollama --version)"
else
    curl -fsSL https://ollama.com/install.sh | sh
    echo "  Ollama installed."
fi

# Start Ollama service if not running
if ! pgrep -x "ollama" > /dev/null; then
    echo "  Starting Ollama server..."
    nohup ollama serve > /dev/null 2>&1 &
    sleep 3
fi

# Pull the default model
echo "  Pulling llama3.1:8b model (this may take a while)..."
ollama pull llama3.1:8b

# --- Piper TTS ---
echo ""
echo "[3/5] Installing Piper TTS..."
mkdir -p "$PIPER_DIR"
mkdir -p "$PIPER_MODELS_DIR"

if [ -f "$PIPER_DIR/piper" ]; then
    echo "  Piper binary already exists."
else
    # Detect architecture
    ARCH=$(uname -m)
    if [ "$ARCH" = "aarch64" ]; then
        PIPER_RELEASE="piper_linux_aarch64.tar.gz"
    elif [ "$ARCH" = "x86_64" ]; then
        PIPER_RELEASE="piper_linux_x86_64.tar.gz"
    else
        echo "  ERROR: Unsupported architecture: $ARCH"
        exit 1
    fi

    echo "  Downloading Piper for $ARCH..."
    PIPER_URL="https://github.com/rhasspy/piper/releases/latest/download/$PIPER_RELEASE"
    wget -q "$PIPER_URL" -O /tmp/piper.tar.gz
    tar -xzf /tmp/piper.tar.gz -C "$PIPER_DIR" --strip-components=1
    rm /tmp/piper.tar.gz
    chmod +x "$PIPER_DIR/piper"
    echo "  Piper installed at $PIPER_DIR/piper"
fi

# Download default voice model
VOICE_MODEL="en_US-lessac-medium"
if [ -f "$PIPER_MODELS_DIR/${VOICE_MODEL}.onnx" ]; then
    echo "  Piper voice model '$VOICE_MODEL' already exists."
else
    echo "  Downloading Piper voice model '$VOICE_MODEL'..."
    VOICE_BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium"
    wget -q "${VOICE_BASE_URL}/${VOICE_MODEL}.onnx" -O "$PIPER_MODELS_DIR/${VOICE_MODEL}.onnx"
    wget -q "${VOICE_BASE_URL}/${VOICE_MODEL}.onnx.json" -O "$PIPER_MODELS_DIR/${VOICE_MODEL}.onnx.json"
    echo "  Voice model downloaded."
fi

# --- Python dependencies ---
echo ""
echo "[4/5] Installing Python dependencies..."
pip3 install -r "$SCRIPT_DIR/requirements.txt"

# --- Verify installation ---
echo ""
echo "[5/5] Verifying installation..."

echo -n "  Ollama: "
if command -v ollama &> /dev/null; then
    echo "OK ($(ollama --version))"
else
    echo "MISSING"
fi

echo -n "  Piper: "
if [ -f "$PIPER_DIR/piper" ]; then
    echo "OK"
else
    echo "MISSING"
fi

echo -n "  Piper model: "
if [ -f "$PIPER_MODELS_DIR/${VOICE_MODEL}.onnx" ]; then
    echo "OK (${VOICE_MODEL})"
else
    echo "MISSING"
fi

echo -n "  faster-whisper: "
python3 -c "import faster_whisper; print('OK')" 2>/dev/null || echo "MISSING"

echo -n "  PyAudio: "
python3 -c "import pyaudio; print('OK')" 2>/dev/null || echo "MISSING"

echo -n "  Ollama server: "
if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "OK (running)"
else
    echo "NOT RUNNING (start with: ollama serve)"
fi

echo -n "  llama3.1:8b model: "
if ollama list 2>/dev/null | grep -q "llama3.1:8b"; then
    echo "OK"
else
    echo "NOT PULLED (run: ollama pull llama3.1:8b)"
fi

echo ""
echo "=========================================="
echo " Setup Complete!"
echo "=========================================="
echo ""
echo " To start the voice assistant:"
echo "   cd $SCRIPT_DIR"
echo "   python3 voice_assistant.py"
echo ""
echo " To start Ollama server (if not running):"
echo "   ollama serve"
echo ""
