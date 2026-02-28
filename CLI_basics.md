# CLI Basics

A friendly introduction to the Command Line Interface (CLI).

The CLI lets you talk directly to your computer using text commands. It can feel technical at first, but once you learn a few basics, it becomes one of the fastest ways to work.

## 1) Core Navigation and File Commands

### `pwd`
- **Meaning:** print working directory
- **Use:** shows your current folder path
- **Example:**
```bash
pwd
```

### `cd`
- **Meaning:** change directory
- **Use:** move between folders
- **Examples:**
```bash
cd projects
cd ..
cd ~
```

### `mkdir`
- **Meaning:** make directory
- **Use:** create a new folder
- **Example:**
```bash
mkdir notes
```

### `cp`
- **Meaning:** copy files/folders
- **Use:** duplicate files or directories
- **Examples:**
```bash
cp file.txt file_backup.txt
cp -r my_folder my_folder_copy
```

### `mv`
- **Meaning:** move (or rename)
- **Use:** move files/folders or rename them
- **Examples:**
```bash
mv old.txt new.txt
mv report.txt ./archive/
```

### `rm`
- **Meaning:** remove
- **Use:** delete files/folders
- **Examples:**
```bash
rm temp.txt
rm -r old_folder
```
- Be careful: deleted files usually do **not** go to trash.

### `cat`
- **Meaning:** concatenate (commonly used to print file content)
- **Use:** quickly view text files
- **Example:**
```bash
cat README.md
```

### `nano`
- **Meaning:** terminal text editor
- **Use:** edit files directly in terminal
- **Example:**
```bash
nano notes.txt
```

### `grep`
- **Meaning:** search text patterns
- **Use:** find words/lines in files
- **Examples:**
```bash
grep "error" app.log
grep -R "TODO" .
```

## 2) Git Basics

Git tracks code changes and supports collaboration.

### Common commands
```bash
git init
git status
git add .
git commit -m "Initial commit"
git log --oneline
git branch
git checkout -b feature/my-change
git switch main
git pull
git push
```

### Typical workflow
1. `git status`
2. Edit files
3. `git add .`
4. `git commit -m "Describe changes"`
5. `git push`

## 3) tmux Basics

`tmux` is a terminal multiplexer: it lets you keep multiple terminal sessions/windows/panes alive.

### Start and attach
```bash
tmux
tmux new -s mysession
tmux ls
tmux attach -t mysession
```

### Useful tmux shortcuts
(Press `Ctrl+b` first, then the key)
- `Ctrl+b` then `c` -> new window
- `Ctrl+b` then `n` -> next window
- `Ctrl+b` then `p` -> previous window
- `Ctrl+b` then `%` -> split pane vertically
- `Ctrl+b` then `"` -> split pane horizontally
- `Ctrl+b` then `o` -> move to next pane
- `Ctrl+b` then `d` -> detach session

## 4) Keyboard Shortcuts You Should Know

### Shell editing/navigation
- `Up Arrow` -> previous command from history
- `Ctrl + Left Arrow` -> move one word left
- `Ctrl + Right Arrow` -> move one word right
- `Ctrl + C` -> cancel running command
- `Ctrl + L` -> clear terminal screen

### Clipboard (most Linux terminal apps)
- `Ctrl + Shift + C` -> copy
- `Ctrl + Shift + V` -> paste

### nano shortcuts
- `Ctrl + O` -> save (write out)
- `Ctrl + X` -> exit
- `Ctrl + K` -> cut line
- `Ctrl + U` -> paste line
- `Ctrl + W` -> search

### tmux note
- Most tmux actions start with the prefix `Ctrl + b`.

## 5) Python in the CLI

### Run Python
```bash
python3
```

### Run a script
```bash
python3 app.py
```

### Create and use a virtual environment
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Install packages
```bash
pip install requests
```

## 6) CMake Basics

CMake helps configure and build C/C++ projects.

### Typical build flow
```bash
cmake -S . -B build
cmake --build build
```

### Run a built program
```bash
./build/my_program
```

### Build type example
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

## Quick Tips
- Use `Tab` for auto-completion.
- Use `history` to see past commands.
- Start simple: practice `pwd`, `cd`, and `ls` until navigation feels natural.
- Be cautious with `rm` and Git branch switching.

You do not need to memorize everything at once. Learn a few commands, use them daily, and your speed will improve quickly.
