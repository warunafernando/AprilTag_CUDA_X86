# Push Instructions

The repository is ready to push. You need to authenticate with GitHub first.

## Steps to Push

### 1. Authenticate with GitHub

You have two options:

**Option A: Use Personal Access Token (Recommended)**
```bash
cd /home/nav/Apriltag/StandAlone
git remote set-url origin https://warunafernando@github.com/warunafernando/ApriTag_Cuda_Full.git
# When prompted, use your GitHub Personal Access Token as the password
```

**Option B: Use SSH (if you have SSH keys set up)**
```bash
cd /home/nav/Apriltag/StandAlone
git remote set-url origin git@github.com:warunafernando/ApriTag_Cuda_Full.git
```

### 2. Rename Existing Main Branch to Old

First, push the existing main branch to "old":
```bash
git fetch origin
git push origin origin/main:old
```

### 3. Push New Code to Main

```bash
git push -f origin main
```

## Complete Command Sequence

If using Personal Access Token:
```bash
cd /home/nav/Apriltag/StandAlone

# Set remote URL with username
git remote set-url origin https://warunafernando@github.com/warunafernando/ApriTag_Cuda_Full.git

# Fetch existing branches
git fetch origin

# Push existing main to old branch
git push origin origin/main:old

# Force push new main branch
git push -f origin main
```

## Creating a Personal Access Token

If you don't have a Personal Access Token:

1. Go to GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Click "Generate new token (classic)"
3. Give it a name (e.g., "ApriTag_Cuda_Full")
4. Select scopes: `repo` (full control of private repositories)
5. Click "Generate token"
6. Copy the token and use it as the password when pushing

## Current Status

- ✅ Git repository initialized
- ✅ All source files committed
- ✅ Remote configured: https://github.com/warunafernando/ApriTag_Cuda_Full.git
- ✅ Git user configured
- ⏳ Waiting for authentication to push

## What Will Be Pushed

- Source code (src/apriltags_cuda/, src/apriltag_cgpadwick/)
- Documentation (README.md, GIT_SETUP.md, etc.)
- Configuration files (.gitignore, .gitattributes)
- Setup scripts (setup_env.sh)

## What Will NOT Be Pushed (excluded via .gitignore)

- Build artifacts (build/ directories)
- Output videos (*.avi files)
- Compiled binaries (*.so, *.a, *.o)
- CMake cache files
- Temporary files



