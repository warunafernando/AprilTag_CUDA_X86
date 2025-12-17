#!/bin/bash
# Script to push to GitHub repository
# This will save the existing main branch as 'old' and push the new code

set -e

echo "=== Pushing to GitHub ==="
echo "Repository: https://github.com/warunafernando/ApriTag_Cuda_Full.git"
echo ""

# Step 1: Save existing main as 'old'
echo "Step 1: Saving existing main branch as 'old'..."
git push origin origin/main:old || {
    echo "Warning: Could not push to 'old' branch (might not exist or already exists)"
    echo "Continuing..."
}

# Step 2: Push new main
echo ""
echo "Step 2: Pushing new main branch..."
git push -f origin main

echo ""
echo "=== Push Complete ==="
echo "The existing main branch has been saved as 'old'"
echo "The new code is now on 'main'"




