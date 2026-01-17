# Git Credential Setup for WSL

## Problem

When using WSL, `git push` may hang without asking for credentials because:
1. Git tries to use the Windows Git Credential Manager
2. The credential manager runs in the background and waits for GUI input
3. You don't see the authentication window

## Solution

Configure Git to use the Windows Git Credential Manager properly in WSL.

### Option 1: Use Windows Git Credential Manager (Recommended)

This allows you to use the same credentials as Windows Git:

```bash
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/bin/git-credential-manager.exe"
```

Then, on first push, a Windows authentication window will appear. After that, credentials are cached.

### Option 2: Use Git Credential Manager Core (Modern)

If you have Git Credential Manager Core installed on Windows:

```bash
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/libexec/git-core/git-credential-manager.exe"
```

### Option 3: Store Credentials in WSL

Store credentials directly in WSL (less secure, but simpler):

```bash
# Store credentials in plain text (use with caution)
git config --global credential.helper store

# Or use cache (credentials expire after timeout)
git config --global credential.helper cache
git config --global credential.cache.timeout 3600  # 1 hour
```

### Option 4: Use SSH instead of HTTPS

The most secure option - use SSH keys:

1. Generate SSH key in WSL:
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

2. Add SSH key to GitHub:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   # Copy the output and add it to GitHub Settings → SSH Keys
   ```

3. Change remote URL to SSH:
   ```bash
   git remote set-url origin git@github.com:waterben/LineExtraction.git
   ```

4. Test connection:
   ```bash
   ssh -T git@github.com
   ```

5. Now push works:
   ```bash
   git push -u origin feature/improved-wsl-support
   ```

## Why Git Doesn't Ask for Credentials Interactively

Git's HTTPS authentication requires a credential helper. Without a properly configured helper:
- Git tries to use the default helper from Windows
- The Windows credential manager runs in background mode
- No interactive prompt appears in the WSL terminal
- The command hangs waiting for input that never comes

## Current Setup Check

Check your current credential helper:

```bash
git config --global credential.helper
```

If it shows a Windows path that's not working, reconfigure using one of the options above.

## Recommended Setup for WSL

For the best experience, we recommend **Option 4 (SSH)**:

```bash
# Generate SSH key
ssh-keygen -t ed25519 -C "$(git config user.email)"

# Start SSH agent
eval "$(ssh-agent -s)"

# Add key to agent
ssh-add ~/.ssh/id_ed25519

# Display public key to add to GitHub
cat ~/.ssh/id_ed25519.pub

# Change to SSH remote
git remote set-url origin git@github.com:waterben/LineExtraction.git

# Test
ssh -T git@github.com
git push -u origin feature/improved-wsl-support
```

## Troubleshooting

### Permission denied (publickey)

If using SSH and getting permission denied:

```bash
# Check SSH agent is running
eval "$(ssh-agent -s)"

# Add your key
ssh-add ~/.ssh/id_ed25519

# Verify key is loaded
ssh-add -l
```

### Windows credential manager not accessible

If the Windows credential manager path doesn't work:

```bash
# Find the correct path
ls -la /mnt/c/Program\ Files/Git/mingw64/bin/git-credential-manager*
ls -la /mnt/c/Program\ Files/Git/mingw64/libexec/git-core/git-credential-manager*

# Use the path that exists
```

### Still hangs on push

If it still hangs:

1. Kill the process: `Ctrl+C`
2. Check what's running:
   ```bash
   ps aux | grep git
   ```
3. Kill any hung git-credential processes:
   ```bash
   pkill -f git-credential
   ```
4. Try again with verbose output:
   ```bash
   GIT_TRACE=1 GIT_CURL_VERBOSE=1 git push -u origin feature/improved-wsl-support
   ```

## Auto-Setup in .vscode_profile

You can add SSH agent auto-start to your `.vscode_profile`:

```bash
# Add to ~/.vscode_profile
if [ -z "$SSH_AUTH_SOCK" ]; then
    eval "$(ssh-agent -s)" >/dev/null 2>&1
    ssh-add ~/.ssh/id_ed25519 2>/dev/null
fi
```

This ensures the SSH agent is running and your key is loaded automatically.
