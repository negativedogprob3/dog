#!/bin/bash

# Unitree Go2 Server Launcher
# Automatically sets up the correct Python environment and runs the server

# Set up environment
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"

# Find pyenv Python 3.11
if command -v pyenv >/dev/null 2>&1; then
    PYTHON_CMD=$(pyenv which python3 2>/dev/null)
    if [ $? -eq 0 ]; then
        echo "🚀 Starting server with Python 3.11: $PYTHON_CMD"
        exec "$PYTHON_CMD" server.py "$@"
    else
        echo "❌ Could not find Python 3.11 via pyenv"
        echo "   Run 'pyenv global 3.11' first"
        exit 1
    fi
else
    echo "❌ pyenv not found - install with 'brew install pyenv' and run setup"
    echo "   See CLAUDE.md for complete setup instructions"
    exit 1
fi