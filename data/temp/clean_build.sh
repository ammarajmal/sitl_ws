#!/bin/bash
WORKSPACE_PATH = "/home/sitl1/sitl_ws"


# Check if the devel directory exists
if [ -d "$WORKSPACE_PATH/devel" ]; then
    # Clean up any existing cmake.lock files
    find "$WORKSPACE_PATH" -name 'cmake.lock' -exec rm -f {} \;
else
    echo "Directory $WORKSPACE_PATH/devel does not exist. Initializing workspace."
    cd "$WORKSPACE_PATH"
    catkin init
fi

# Add cmake.lock to .gitignore if not already present
GITIGNORE_PATH="$WORKSPACE_PATH/.gitignore"
if [ ! -f "$GITIGNORE_PATH" ]; then
    touch "$GITIGNORE_PATH"
fi

if ! grep -q "**/cmake.lock" "$GITIGNORE_PATH"; then
    echo "**/cmake.lock" >> "$GITIGNORE_PATH"
fi

# Run the catkin build
cd "$WORKSPACE_PATH"
catkin build