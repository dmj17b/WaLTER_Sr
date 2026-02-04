#!/bin/bash
set -e

# ============================================================================
# Configuration
# ============================================================================

# Path to your private deploy key (REQUIRED)
DEPLOY_KEY_PATH="${HOME}/.ssh/id_rsa"

# Path to staged bag data/local backup
STAGE_PATH="${HOME}/bag_data"

# Location of the Git repository where experiment logs are stored
GIT_PATH="${HOME}/experiment_logs"

# GitHub repository SSH URL (find on GitHub under "Code" -> "SSH")
SSH_URL="git@github.com:USERNAME/REPOSITORY.git"

# Git branch to push to
BRANCH_NAME="main"

# ============================================================================
# Validation
# ============================================================================

echo "Starting experiment setup..."

# Check if deploy key exists
# if [[ ! -f "$DEPLOY_KEY_PATH" ]]; then
# 	echo "ERROR: Deploy key not found at: $DEPLOY_KEY_PATH"
# 	echo "Please ensure the path is correct and the file exists."
# 	exit 1
# fi

# Set correct permissions for the private key
# chmod 600 "$DEPLOY_KEY_PATH"

# Check if stage path exists
if [[ ! -d "$STAGE_PATH" ]]; then
	echo "WARNING: Stage path does not exist. Creating: $STAGE_PATH"
	mkdir -p "$STAGE_PATH"
fi

# Check if git repository exists
# if [[ ! -d "$GIT_PATH" ]]; then
# 	echo "ERROR: Git repository path does not exist: $GIT_PATH"
# 	exit 1
# fi

if source install/setup.bash; then
	echo "Sourced ROS2 workspace successfully."
else
	echo "ERROR: Failed to source ROS2 workspace. Ensure the path is correct."
	exit 1
fi

# ============================================================================
# Determine Experiment Name
# ============================================================================

python3 ./scripts/get_experiment_name.py

# Retrieve the experiment name from temporary file
EXPERIMENT_NAME_FILE="${HOME}/.experiment_name"
if [[ ! -f "$EXPERIMENT_NAME_FILE" ]]; then
	echo "ERROR: Experiment name file not created by Python script."
	exit 1
fi

EXPERIMENT_NAME=$(cat "$EXPERIMENT_NAME_FILE")
rm "$EXPERIMENT_NAME_FILE"

if [[ -z "$EXPERIMENT_NAME" ]]; then
	echo "ERROR: Python script generated an empty experiment name."
	exit 1
fi

echo "Experiment name: ${EXPERIMENT_NAME}"

# Create full experiment name with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M")
EXPERIMENT_NAME_FULL="${TIMESTAMP}_${EXPERIMENT_NAME}"
SAVE_LOCATION="${STAGE_PATH}/${EXPERIMENT_NAME_FULL}"

# ============================================================================
# Run Experiment
# ============================================================================

echo "Running experiment: ${EXPERIMENT_NAME}"
ros2 launch launch/experiment_launch.py save_location:="${SAVE_LOCATION}"

# ============================================================================
# Post-Processing
# ============================================================================

echo "Converting bag files to CSV..."
python3 ./scripts/rosbag2csv.py "$SAVE_LOCATION"

# Rename CSV files with experiment name prefix
echo "Renaming CSV files..."
shopt -s nullglob # Prevent loop from running if no files match
for file in "${SAVE_LOCATION}"/*.csv; do
	file_name=$(basename "$file")
	new_name="${SAVE_LOCATION}/${EXPERIMENT_NAME}_${file_name}"
	echo "  $file_name -> ${EXPERIMENT_NAME}_${file_name}"
	mv "$file" "$new_name"
done
shopt -u nullglob

# ============================================================================
# Git Operations
# ============================================================================

# echo "Copying experiment data to Git repository..."
# cp -r "$SAVE_LOCATION" "$GIT_PATH"

# cd "$GIT_PATH"

# # Verify we're in a Git repository
# if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
# 	echo "ERROR: $GIT_PATH is not a Git repository."
# 	exit 1
# fi

# # Set remote URL
# git remote set-url origin "${SSH_URL}"

# # Stage all changes
# git add .
# echo "Staged all changes"

# # Commit changes
# TIMESTAMP2=$(date +"%Y-%m-%d %H:%M:%S")
# COMMIT_MESSAGE="Experiment ${EXPERIMENT_NAME} completed at ${TIMESTAMP2}"

# if git diff --cached --quiet; then
# 	echo "No changes to commit. Skipping commit and push."
# 	exit 0
# fi

# git commit -m "$COMMIT_MESSAGE"
# echo "Committed: \"$COMMIT_MESSAGE\""

# # Push using specific deploy key
# echo "Pushing to origin/$BRANCH_NAME..."
# if GIT_SSH_COMMAND="ssh -i $DEPLOY_KEY_PATH -o IdentitiesOnly=yes -o StrictHostKeyChecking=no" git push origin "$BRANCH_NAME"; then
# 	echo "✓ DEPLOYMENT SUCCESSFUL!"
# else
# 	echo "✗ DEPLOYMENT FAILED during push."
# 	exit 1
# fi

# echo "Experiment pipeline completed successfully."
# exit 0
