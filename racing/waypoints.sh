#!/bin/bash

# Check if the DOCKER_CONT_ID argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 DOCKER_CONT_ID"
    exit 1
fi

DOCKER_CONT_ID="$1"

# List of files to copy from the container
FILES_TO_COPY=(
    "/occupancy_grid.npy"
    "/map_metadata.npy"
    "/map_origin.npy"
    "/pose.npy"
)

# Destination directory on the host machine
DEST_DIR="/path/on/host"

# Loop through the list of files and copy them from the container to the host
for FILE_PATH in "${FILES_TO_COPY[@]}"; do
    docker cp "$DOCKER_CONT_ID:$FILE_PATH" "$DEST_DIR"
    if [ $? -eq 0 ]; then
        echo "Copied $FILE_PATH from container to $DEST_DIR"
    else
        echo "Failed to copy $FILE_PATH from container"
    fi
done
