# Git and GitHub: A Beginner's Guide

---

## Table of Contents

1. [Introduction](#introduction)
2. [Before You Start](#before-you-start)
3. [What is Git?](#what-is-git)
4. [What is GitHub?](#what-is-github)
5. [Why We Use Git in This Course](#why-we-use-git-in-this-course)
6. [The 4-Step Workflow](#the-4-step-workflow)
7. [First-Time Setup](#first-time-setup)
8. [Cloning a Repository](#cloning-a-repository)
9. [Creating a Branch](#creating-a-branch)
10. [Making Changes](#making-changes)
11. [Checking Status](#checking-status)
12. [Staging Your Files](#staging-your-files)
13. [Committing Your Changes](#committing-your-changes)
14. [Pushing to GitHub](#pushing-to-github)
15. [How to Submit Your Work in This Course](#how-to-submit-your-work-in-this-course)
16. [Pulling Updates Before Starting Work](#pulling-updates-before-starting-work)
17. [Common Beginner Mistakes](#common-beginner-mistakes)
18. [Troubleshooting Quick Fixes](#troubleshooting-quick-fixes)
19. [Quick Reference Table](#quick-reference-table)

---

## Introduction

Welcome. This guide will teach you everything you need to use Git and GitHub in this course. You do not need any prior experience.

**What you will be able to do after finishing this guide:**

- Clone a course repository to your computer
- Create your own working branch
- Save and describe your changes with commits
- Push your work to GitHub
- Submit your assignment for review

**Estimated time:** 30–45 minutes to read and practice.

There is no advanced theory here. This is a practical, step-by-step guide. Follow it in order and you will be ready to submit your first assignment.

---

## Before You Start

Make sure you have the following ready before continuing.

- [ ] Git is installed on your computer
- [ ] You have a GitHub account (free at [github.com](https://github.com))
- [ ] You can open a terminal (on Ubuntu: press `Ctrl + Alt + T`)
- [ ] You have an internet connection

To confirm Git is installed, open your terminal and run:

```bash
git --version
```

✅ If you see a version number (for example, `git version 2.43.0`), Git is installed and ready.

---

## What is Git?

Think of Git as an automatic save system for your code — like saving checkpoints in a video game.

Every time you reach a good stopping point, you save a snapshot of your work. If something goes wrong later, you can always go back to a previous snapshot. Git stores all of these snapshots on your computer and keeps a full history of every change you have made.

You do not need to understand how Git works internally. You just need to know a few commands to use it effectively.

---

## What is GitHub?

GitHub is a website that stores your Git project online so that others can see it and collaborate with you.

| Git | GitHub |
|-----|--------|
| Runs on your computer | Lives on the internet |
| Tracks changes locally | Hosts your project online |
| Works offline | Requires internet connection |
| Used via terminal commands | Used via a web browser |

Think of Git as your personal save system and GitHub as the shared cloud where your team can access your work.

---

## Why We Use Git in This Course

Every assignment in this course is submitted through GitHub. You will:

1. Receive a repository link for each assignment
2. Clone that repository to your computer
3. Do your work on your own branch
4. Push your work to GitHub
5. Submit a Pull Request for instructor review

This is the standard workflow used by software teams around the world. Learning it now will serve you throughout your career.

---

## The 4-Step Workflow

Every time you work on an assignment, you follow these four steps:

```
1. BRANCH   →   Create your own working space
2. CHANGE   →   Edit your files
3. COMMIT   →   Save a snapshot with a description
4. PUSH     →   Upload your work to GitHub
```

Here is how the flow looks in terms of commands:

```
Your Files
    ↓  git add
Staging Area
    ↓  git commit
Local Repository
    ↓  git push
GitHub (Remote)
```

You will learn each of these steps in detail below.

---

## First-Time Setup

You only need to do this once. It tells Git who you are so your name appears on your commits.

Open your terminal and run the following two commands. Replace the name and email with your own:

```bash
git config --global user.name "Your Name"
```

```bash
git config --global user.email "your@email.com"
```

To confirm your setup worked:

```bash
git config --list
```

✅ You should see your name and email listed in the output.

💡 Use your real name and your university email address. This information will appear on every commit you make.

---

## Cloning a Repository

Cloning means downloading a copy of a GitHub project to your computer.

For each assignment, your instructor will provide a GitHub link. Use it like this:

```bash
git clone https://github.com/your-course/assignment-repo.git
```

Then move into the project folder:

```bash
cd assignment-repo
```

✅ If you see a new folder appear with the project files, the clone was successful.

💡 You only clone once per project. After that, the folder lives on your computer and you work inside it.

---

## Creating a Branch

A branch is your own private working space inside the project. You make all your changes there without affecting anyone else's work.

⚠️ **Never work directly on the `main` branch.** Always create your own branch first.

To create a new branch and switch to it:

```bash
git checkout -b feature/your-name-task-name
```

For example:

```bash
git checkout -b feature/sara-login-page
```

✅ Git will confirm: `Switched to a new branch 'feature/sara-login-page'`

To check which branch you are currently on at any time:

```bash
git branch
```

The branch with the `*` symbol next to it is your active branch.

---

## Making Changes

Now open the project folder in your code editor (VS Code, Gedit, or any editor you prefer) and make your changes.

You can:

- Create new files
- Edit existing files
- Delete files that are no longer needed

There is nothing special to do while editing. Just work normally. Git is watching for changes in the background.

💡 You cannot break GitHub by making changes locally. Until you push, nothing on GitHub is affected.

---

## Checking Status

After making changes, always check what Git has noticed:

```bash
git status
```

This command shows you:

- Which files you have changed (shown in red — not yet staged)
- Which files are ready to be committed (shown in green — staged)

✅ If you see your changed files listed in red, everything is working correctly. It means Git detected your changes and is waiting for you to stage them.

Run `git status` often. It is completely safe and gives you a clear picture of where you are.

---

## Staging Your Files

Staging means selecting which changes you want to include in your next commit.

To stage a specific file:

```bash
git add filename.txt
```

To stage all changed files at once:

```bash
git add .
```

After staging, run `git status` again:

```bash
git status
```

✅ Your files should now appear in green. They are staged and ready to be committed.

---

## Committing Your Changes

A commit is a saved snapshot of your staged changes. Every commit includes a short message describing what you did.

```bash
git commit -m "Add login form to homepage"
```

Write your message inside the quotation marks. A good commit message is short and specific.

**Good commit messages:**

- `"Add user registration page"`
- `"Fix navigation bar alignment"`
- `"Update README with setup instructions"`

**Avoid vague messages like:**

- `"fixed stuff"`
- `"changes"`
- `"update"`

💡 A good commit message should complete this sentence: *"This commit will ___."*

✅ After committing, Git will show a summary confirming the commit was saved, including the branch name and a short ID.

---

## Pushing to GitHub

Pushing uploads your local commits to GitHub so they are visible online and ready for review.

```bash
git push origin feature/your-branch-name
```

For example:

```bash
git push origin feature/sara-login-page
```

✅ If the push succeeds, Git will display a URL. You can open that link directly to create your Pull Request on GitHub.

⚠️ If your push is rejected, it usually means someone else pushed changes before you. See [Troubleshooting Quick Fixes](#troubleshooting-quick-fixes) below for the solution.

---

## How to Submit Your Work in This Course

Each assignment follows this exact submission process.

### Step 1 — Receive the assignment

Your instructor will share a GitHub repository link for the assignment.

### Step 2 — Clone the repository

```bash
git clone https://github.com/your-course/assignment-repo.git
cd assignment-repo
```

### Step 3 — Create your branch

Always name your branch using this format:

```
feature/your-name-task-name
```

Example:

```bash
git checkout -b feature/sara-login-page
```

### Step 4 — Do your work

Edit files, create new ones, and complete the assignment requirements.

### Step 5 — Stage and commit your changes

```bash
git add .
git commit -m "Complete login page assignment"
```

### Step 6 — Push your branch

```bash
git push origin feature/sara-login-page
```

### Step 7 — Create a Pull Request on GitHub

1. Go to the repository on GitHub
2. Click **"Compare & pull request"** (GitHub shows this automatically after you push)
3. Write a short description of what you did
4. Click **"Create pull request"**
5. Wait for your instructor's review — do not merge it yourself

✅ Once your Pull Request is submitted, your assignment is officially turned in.

💡 Your description should briefly explain what you did and anything the reviewer should know.

---

## Pulling Updates Before Starting Work

Before you start any new work session, always download the latest version of the project. This prevents conflicts with changes your teammates or instructor may have pushed.

```bash
git checkout main
git pull origin main
```

Then create your new branch from this updated version:

```bash
git checkout -b feature/your-name-new-task
```

⚠️ Skipping this step is one of the most common causes of merge conflicts. Make it a habit to pull before you start.

---

## Common Beginner Mistakes

These are the mistakes almost everyone makes when starting with Git. Knowing them in advance will save you time.

**Working directly on `main`**
Always create a branch first. Never commit to `main`.

**Forgetting to pull before starting**
Run `git pull origin main` at the start of every work session.

**Forgetting to push before submitting**
Your instructor cannot see your work until you push it to GitHub. Always push before creating your Pull Request.

**Using the wrong branch name**
Follow the naming format exactly: `feature/your-name-task-name`. Inconsistent names cause confusion during review.

**Writing unclear commit messages**
`"fixed stuff"` tells nobody anything. Write `"Fix button alignment on login page"` instead.

**Not checking status before committing**
Always run `git status` before `git add` and before `git commit`. It takes two seconds and prevents many problems.

---

## Troubleshooting Quick Fixes

### Problem: "fatal: not a git repository"

You are running a Git command outside of a project folder.

**Solution:** Navigate into your cloned project folder first:

```bash
cd assignment-repo
```

---

### Problem: Push rejected — "non-fast-forward"

Someone pushed changes to GitHub before you did.

**Solution:** Pull the latest changes first, then push again:

```bash
git pull origin main
git push origin feature/your-branch-name
```

💡 This is normal and nothing went wrong. Git is protecting you from overwriting someone else's work.

---

### Problem: You committed to the wrong branch

**Solution:** If you have not pushed yet, you can switch branches without losing your work. First check which commit you need:

```bash
git log --oneline
```

Note the commit ID (the short code on the left). Then switch to the correct branch and apply the commit:

```bash
git checkout correct-branch-name
git cherry-pick commit-id
```

⚠️ Prevention is better: always check your current branch with `git branch` before committing.

---

### Problem: You forgot to stage files before committing

**Solution:** Stage them now and commit again:

```bash
git add .
git commit -m "Add missing files"
```

---

### Problem: Merge conflict markers in your file

When you see `<<<<<<<`, `=======`, and `>>>>>>>` inside a file, Git found a conflict it could not resolve automatically.

**Solution:**

1. Open the file in your editor
2. Find the conflict markers
3. Choose which version to keep and delete the other version
4. Remove the conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`)
5. Save the file, then run:

```bash
git add filename
git commit -m "Resolve merge conflict in filename"
```

💡 Merge conflicts look alarming but they are straightforward to fix. Read the file carefully and choose the correct version.

---

## Quick Reference Table

| Task | Command |
|------|---------|
| Check Git version | `git --version` |
| Configure name | `git config --global user.name "Your Name"` |
| Configure email | `git config --global user.email "email"` |
| Clone a repository | `git clone <url>` |
| Enter project folder | `cd folder-name` |
| Check current branch | `git branch` |
| Create and switch to new branch | `git checkout -b branch-name` |
| Check file status | `git status` |
| Stage all files | `git add .` |
| Stage a specific file | `git add filename` |
| Commit with message | `git commit -m "message"` |
| Push to GitHub | `git push origin branch-name` |
| Pull latest changes | `git pull origin main` |
| View commit history | `git log --oneline` |

---

*Mistakes are a normal part of learning Git. Every developer has accidentally committed to the wrong branch or forgotten to push. The key is practicing the workflow consistently until it becomes second nature.*
