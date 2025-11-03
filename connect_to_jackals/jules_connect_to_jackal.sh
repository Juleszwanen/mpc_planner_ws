#!/bin/bash
source devel/setup.bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Usage: source jules_connect_to_jackal.sh <jackal_name>"
    echo "Available options: jackal1, jackal2, jackal3, jackal4"
    echo "Example: source jules_connect_to_jackal.sh jackal1"
    return 1
fi

JACKAL_NAME=$1

case $JACKAL_NAME in
    jackal1)
        # Jackal 1
        export ROS_MASTER_URI=http://192.168.0.101:11311
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

# Your IP (check with `ip a`)
export ROS_IP=192.168.0.99
echo "ROS_IP set to: $ROS_IP"