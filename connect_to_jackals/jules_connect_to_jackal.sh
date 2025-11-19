#!/bin/bash
source devel/setup.bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Usage: source jules_connect_to_jackal.sh <jackal_name> [computer_name]"
    echo "Available jackals: jackal1, jackal2, jackal3, jackal4"
    echo "Available computers: tu_delft, jules"
    echo "Example: source jules_connect_to_jackal.sh jackal1 jules"
    echo ""
    echo "If computer_name is omitted, will auto-detect based on your IP address"
    return 1
fi

JACKAL_NAME=$1
COMPUTER_NAME=$2

# Auto-detect computer if not specified
if [ -z "$COMPUTER_NAME" ]; then
    CURRENT_IP=$(hostname -I | awk '{print $1}')
    if [[ "$CURRENT_IP" == 192.168.3.* ]]; then
        COMPUTER_NAME="tu_delft"
        echo "Auto-detected computer: tu_delft (IP: $CURRENT_IP)"
    elif [[ "$CURRENT_IP" == 192.168.0.* ]]; then
        COMPUTER_NAME="jules"
        echo "Auto-detected computer: jules (IP: $CURRENT_IP)"
    else
        echo "Warning: Could not auto-detect computer from IP $CURRENT_IP"
        echo "Please specify computer name: source jules_connect_to_jackal.sh $JACKAL_NAME <computer_name>"
        return 1
    fi
fi

case $JACKAL_NAME in
    jackal1)
        # Jackal 1
        export ROS_MASTER_URI=http://cpr-j100-0114:11311
        # export ROS_MASTER_URI=http://192.168.0.101:11311
        echo "Connected to Jackal 1: $ROS_MASTER_URI"
        ;;
    jackal2)
        # Jackal 2
        export ROS_MASTER_URI=http://192.168.0.102:11311
        echo "Connected to Jackal 2: $ROS_MASTER_URI"
        ;;
    jackal3)
        # Jackal 3
        export ROS_MASTER_URI=http://192.168.0.103:11311
        echo "Connected to Jackal 3: $ROS_MASTER_URI"
        ;;
    jackal4)
        # Jackal 4
        export ROS_MASTER_URI=http://192.168.0.104:11311
        echo "Connected to Jackal 4: $ROS_MASTER_URI"
        ;;
    *)
        echo "Error: Unknown jackal name '$JACKAL_NAME'"
        echo "Available options: jackal1, jackal2, jackal3, jackal4"
        return 1
        ;;
esac

# Set ROS_IP based on computer
case $COMPUTER_NAME in
    tu_delft)
        export ROS_IP=192.168.3.56
        echo "Computer: TU Delft (ROS_IP: $ROS_IP)"
        ;;
    jules)
        export ROS_IP=192.168.0.99
        echo "Computer: Jules (ROS_IP: $ROS_IP)"
        ;;
    *)
        echo "Error: Unknown computer name '$COMPUTER_NAME'"
        echo "Available options: tu_delft, jules"
        return 1
        ;;
esac

echo ""
echo "✓ ROS Configuration Complete:"
echo "  • Jackal: $JACKAL_NAME"
echo "  • Master URI: $ROS_MASTER_URI"
echo "  • Your IP: $ROS_IP"
echo ""