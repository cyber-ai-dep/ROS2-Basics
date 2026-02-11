# Practical Practise 1 — Solution

## Target Structure

```
/project_workspace
├── notes
├── main.txt
└── code
    ├── java
    │   └── Hello.java
    └── bash
        └── script.sh

4 directories, 3 files
```

Then:

- Adding `"This is the main project file"` to `main.txt`
- Adding `'public class Hello {}'` to `Hello.java`

---

## Solution

### 1 Create the Structure

```bash
mkdir -p project_workspace/code/{java,bash} project_workspace/notes
touch project_workspace/main.txt
touch project_workspace/code/java/Hello.java
touch project_workspace/code/bash/script.sh
```

### 2 Add Content

```bash
cd project_workspace
echo "This is the main project file" > main.txt
echo 'public class Hello {}' > code/java/Hello.java
```
