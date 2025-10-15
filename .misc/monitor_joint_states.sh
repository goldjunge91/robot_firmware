#!/bin/bash

# Joint States Monitor Script
# Displays motor encoder feedback in a readable format

set -e

# Colors
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Joint States Monitor${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Monitoring /joint_states topic..."
echo "Press Ctrl+C to stop"
echo ""
echo -e "${YELLOW}Motor Layout:${NC}"
echo "  [0] Front Left    [1] Front Right"
echo "  [2] Rear Left     [3] Rear Right"
echo ""

# Monitor joint states with formatted output
ros2 topic echo /joint_states --field velocity
