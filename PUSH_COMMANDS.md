# Commands to Push to GitHub

Since authentication is configured in Cursor, run these commands in **Cursor's integrated terminal**:

## Step 1: Save existing main branch as 'old'

```bash
cd /home/nav/Apriltag/StandAlone
git push origin origin/main:old
```

## Step 2: Push new main branch

```bash
git push -f origin main
```

## Alternative: Use Cursor's Git UI

1. Open Source Control panel (Ctrl+Shift+G or Cmd+Shift+G)
2. You should see your commit ready to push
3. Click the "..." menu (three dots)
4. Select "Push" → "Push to..."
5. Choose "origin" and branch "main"
6. Click "OK"

**Note**: To save the old main branch first, you'll need to run the first command in the terminal before using the UI.

## Current Status

- ✅ 1 commit ready to push
- ✅ Branch: main
- ✅ Remote: https://github.com/warunafernando/ApriTag_Cuda_Full.git
- ✅ 212 files committed (build artifacts excluded)




