# Practical Practise 2 — Solution

## Tasks

### 1️ Copy files (`cp`)

- Copy `main.txt` into the `notes` directory.
- Copy `Hello.java` into the `notes` directory.
- Do not delete the original files.

### 2️ Move & rename files (`mv`)

- Move `script.sh` from `code/bash` to the `notes` directory.
- Rename `script.sh` to `backup.sh`.

### 3️ Remove files (`rm`)

- Delete the copied `Hello.java` from the `notes` directory.
- Delete `backup.sh` from the `notes` directory.

> ⚠️ **Be careful:** Do not delete the original `Hello.java` inside `code/java`.

---

## Solution

### 1 Copy files

```bash
cd ~/project_workspace
cp main.txt notes/
cp code/java/Hello.java notes/
```

### 2 Move & rename

```bash
mv code/bash/script.sh notes/backup.sh
```

### 3 Remove files

```bash
rm notes/Hello.java
rm notes/backup.sh
```

---

## Verification

```bash
ls notes
ls code/java
ls
```

**Expected:**

- `notes` → contains only `main.txt`
- `code/java/Hello.java` still exists
- `main.txt` still exists in the main directory
