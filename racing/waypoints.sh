#!/bin/bash

# Check if DOCKER_CONT_ID argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 DOCKER_CONT_ID"
    exit 1
fi

DOCKER_CONT_ID="$1"
DEST_DIR="./waypoints_dir"

# List of files to copy from the container
FILES_TO_COPY=(
    "/occupancy_grid.npy"
    "/map_metadata.npy"
    "/map_origin.npy"
    "/pose.npy"
)

# Loop through the list of files and copy them from the container to the host
for FILE_PATH in "${FILES_TO_COPY[@]}"; do
    docker cp "$DOCKER_CONT_ID:$FILE_PATH" "$DEST_DIR"
    if [ $? -eq 0 ]; then
        echo "Copied $FILE_PATH from container to $DEST_DIR"
    else
        echo "Failed to copy $FILE_PATH from container"
    fi
done

# Run python script
python3 a_star_circuit.py "$DEST_DIR"

# Copy rx.npy and ry.npy files from DEST_DIR back to the container
docker cp "$DEST_DIR/rx.npy" "$DOCKER_CONT_ID:/path/in/container"
docker cp "$DEST_DIR/ry.npy" "$DOCKER_CONT_ID:/path/in/container"

if [ $? -eq 0 ]; then
    echo "Copied rx.npy and ry.npy files back to container"
else
    echo "Failed to copy rx.npy and ry.npy files back to container"
fi