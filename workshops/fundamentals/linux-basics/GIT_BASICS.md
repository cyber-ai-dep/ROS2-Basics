# Git and GitHub Guide for Beginners

## What is Git and GitHub?

**Git** is a version control system that tracks changes in your code. It runs on your computer and lets you save snapshots of your project at different points in time.

**GitHub** is a website that hosts your Git projects online. It allows teams to collaborate, review code, and manage projects together.

### Git vs GitHub

| Git | GitHub |
|-----|--------|
| Software on your computer | Website on the internet |
| Tracks file changes locally | Hosts repositories online |
| Works offline | Requires internet connection |
| Command-line tool | Web interface with collaboration features |
| Manages version history | Enables team workflows and code review |

## Core Concepts

| Term | Simple Meaning | Example |
|------|----------------|---------|
| **Repository** | A project folder tracked by Git | Your course project folder |
| **Clone** | Copy a repository to your computer | Getting the starter code from GitHub |
| **Commit** | A saved snapshot of your changes | "Added login form" |
| **Branch** | A separate line of development | `feature/add-navbar` |
| **Merge** | Combine changes from one branch into another | Merging your feature into main |
| **Pull** | Download changes from GitHub to your computer | Getting your teammate's updates |
| **Push** | Upload your commits to GitHub | Sharing your work with the team |
| **Pull Request** | Request to merge your branch (on GitHub) | "Please review my login feature" |

## Git Areas Explained

Git has three main areas where your code exists:

**Working Directory:** The actual files you see and edit on your computer.

**Staging Area:** A preparation zone where you select which changes to commit.

**Local Repository:** Where Git stores your committed snapshots.

**Remote Repository:** The version on GitHub that your team shares.

### Visual Flow

```
Working Directory  →  Staging Area  →  Local Repository  →  Remote (GitHub)
   (edit files)      (git add)         (git commit)          (git push)
```

**Commands in this flow:**

- `git add` moves changes from Working Directory to Staging Area
- `git commit` saves staged changes to Local Repository
- `git push` uploads commits to Remote Repository on GitHub

## First-Time Setup

Install Git, then configure your identity:

```bash
git --version
```

```bash
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

```bash
git config --list
```

The name and email appear in every commit you make. Use your real name and school email.

## Basic Workflow

### 1. Clone a Repository

```bash
git clone https://github.com/username/repo-name.git
cd repo-name
```

Downloads the project to your computer and enters the folder.

### 2. Create a Branch

```bash
git checkout -b feature/new-feature
```

Creates a new branch and switches to it. Never work directly on `main`.

### 3. Make Changes

Edit files using your text editor or IDE. Add new files, modify existing ones, or delete files as needed.

### 4. Check Status

```bash
git status
```

Shows which files you modified. Red means unstaged, green means staged.

### 5. Stage Changes

```bash
git add filename.txt
```

```bash
git add .
```

Stages specific file or all changed files for the next commit.

### 6. Commit Changes

```bash
git commit -m "Add login form to homepage"
```

Saves your staged changes with a descriptive message.

### 7. Push to GitHub

```bash
git push origin feature/new-feature
```

Uploads your branch to GitHub so others can see it.

### 8. Create Pull Request

Go to GitHub, click "Compare & pull request", add description, and submit for review.

## Working in a Team

### Always Pull Before Starting Work

```bash
git checkout main
git pull origin main
```

This ensures you have the latest code before creating a new branch.

### Someone Pushed Before You

If `git push` fails with "rejected" error:

```bash
git pull origin main
```

Then try pushing again. Git will merge the latest changes with yours.

### What is a Merge Conflict?

A merge conflict happens when you and a teammate edit the same line in the same file. Git cannot decide which version to keep.

**How to fix:**

1. Git marks conflicts in your file with `<<<<<<<`, `=======`, `>>>>>>>`
2. Open the file and choose which version to keep
3. Remove the conflict markers
4. Run `git add filename` and `git commit`

## Best Practices

- **Never work on main directly** – always create a feature branch
- **Pull before starting work** – get the latest changes first
- **Write clear commit messages** – "Add user login" not "fixed stuff"
- **Keep commits small** – one logical change per commit
- **Use .gitignore** – exclude files like `node_modules/`, `.env`, `.DS_Store`
- **Never push secrets** – no passwords, API keys, or tokens in code
- **Push often** – do not wait days to share your work
- **Review your changes** – run `git status` and `git diff` before committing

## Common Commands

```bash
# View status
git status

# View changes
git diff

# Create and switch to branch
git checkout -b branch-name

# Switch to existing branch
git checkout branch-name

# List all branches
git branch

# Stage files
git add filename
git add .

# Commit
git commit -m "message"

# Push
git push origin branch-name

# Pull
git pull origin main

# View commit history
git log --oneline
```

## .gitignore Example

Create a `.gitignore` file in your project root:

```
# Dependencies
node_modules/
venv/

# Environment files
.env
.env.local

# IDE
.vscode/
.idea/

# OS files
.DS_Store
Thumbs.db

# Build output
dist/
build/
```

## Mini Practice Task

**Goal:** Add a login page feature to the project.

### Steps

1. **Clone the repository** (if not already cloned)

```bash
git clone https://github.com/your-course/project.git
cd project
```

2. **Create a feature branch**

```bash
git checkout -b feature/login-page
```

3. **Create a new file** called `login.html` and add basic HTML structure

4. **Check what changed**

```bash
git status
```

5. **Stage your changes**

```bash
git add login.html
```

6. **Commit with a clear message**

```bash
git commit -m "Add login page with form structure"
```

7. **Push to GitHub**

```bash
git push origin feature/login-page
```

8. **Create Pull Request** on GitHub with description: "Adds new login page for user authentication"

### What You Learned

- Creating feature branches
- Staging and committing changes
- Writing meaningful commit messages
- Pushing branches to GitHub
- Opening pull requests for code review

## Quick Reference Table

| Task | Command |
|------|---------|
| Check Git version | `git --version` |
| Configure name | `git config --global user.name "Name"` |
| Configure email | `git config --global user.email "email"` |
| Clone repository | `git clone <url>` |
| Check status | `git status` |
| Create branch | `git checkout -b branch-name` |
| Switch branch | `git checkout branch-name` |
| Stage files | `git add .` |
| Commit | `git commit -m "message"` |
| Push | `git push origin branch-name` |
| Pull | `git pull origin main` |
| View history | `git log --oneline` |

## Troubleshooting

### Problem: "fatal: not a git repository"

**Solution:** You are not inside a Git project folder. Run `git clone` or navigate to the correct directory.

### Problem: "rejected - non-fast-forward"

**Solution:** Someone pushed changes before you. Run `git pull origin main` then try pushing again.

### Problem: Commit to wrong branch

**Solution:** If you have not pushed yet, switch to the correct branch and bring your changes with you:

```bash
git checkout correct-branch
git cherry-pick commit-hash
```

Better practice: always check your current branch with `git branch` before committing.

## Summary

Git workflow in four steps:

1. **Branch** – create a feature branch
2. **Change** – edit your files
3. **Commit** – save snapshots with messages
4. **Push** – share your work on GitHub

Remember: commit often, push regularly, pull before starting work, and never work directly on main.

## Additional Resources

### Official Documentation

| Resource | Description | Link |
|----------|-------------|------|
| **Git Official Documentation** | Complete reference for all Git commands | [git-scm.com/doc](https://git-scm.com/doc) |
| **GitHub Docs** | Guide to using GitHub features | [docs.github.com](https://docs.github.com) |
| **Git Cheat Sheet** | Quick reference PDF from GitHub | [education.github.com/git-cheat-sheet](https://education.github.com/git-cheat-sheet-education.pdf) |

### Interactive Learning

| Resource | Description | Best For |
|----------|-------------|----------|
| **Learn Git Branching** | Visual interactive Git tutorial | Understanding branching concepts |
| **GitHub Skills** | Hands-on courses directly on GitHub | Practice with real repositories |
| **Git-it** | Desktop app for learning Git | Offline practice |

**Links:**
- Learn Git Branching: [learngitbranching.js.org](https://learngitbranching.js.org)
- GitHub Skills: [skills.github.com](https://skills.github.com)
- Git-it: [github.com/jlord/git-it-electron](https://github.com/jlord/git-it-electron)

### Video Tutorials

| Resource | Length | Description |
|----------|--------|-------------|
| **Git and GitHub for Beginners - freeCodeCamp** | 1 hour | Complete beginner tutorial |
| **Git Tutorial for Beginners - Programming with Mosh** | 1 hour | Clear explanations with examples |
| **GitHub Ultimate Course - Traversy Media** | 40 min | GitHub-focused workflow |

### Practice Repositories

Create a practice repository on GitHub and experiment with:
- Creating branches
- Making commits
- Opening pull requests
- Resolving merge conflicts

**Tip:** You cannot break anything in your own practice repository, so experiment freely.

### Markdown Guide

Since README files use Markdown formatting:

- **Markdown Guide**: [markdownguide.org](https://www.markdownguide.org)
- **GitHub Markdown**: [docs.github.com/en/get-started/writing-on-github](https://docs.github.com/en/get-started/writing-on-github)

### Getting Help

When stuck:

1. **Read error messages carefully** – Git error messages usually explain the problem
2. **Use `git --help`** – Built-in documentation for any command: `git commit --help`
3. **Search on Stack Overflow** – Most Git questions are already answered
4. **Ask your instructor or TA** – They understand the course context



## Next Steps

After mastering these basics:

1. Practice the workflow on small personal projects
2. Contribute to your team's course project
3. Explore GitHub features like Issues and Projects
4. Learn to write better commit messages
5. Understand when and how to use `.gitignore` effectively

**Remember:** Everyone struggles with Git at first. The key is consistent practice and not being afraid to make mistakes in your practice repository.