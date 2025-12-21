## 🛠️ Setup: Changing User Directories

This robotics package uses **absolute file paths** (e.g., in configuration or launch files) that likely include the previous developer's home directory (e.g., `/home/old-user-name/...`). Before building or running the package, you **must** replace all occurrences of the previous user's name with your current Linux username.

### 1\. Identify Usernames

| Placeholder | Meaning | Example Value |
| :--- | :--- | :--- |
| **`OLD_USER`** | The username currently present in the repository's files. | `marwan` |
| **`NEW_USER`** | Your actual Linux username on the current machine. | `robot-dev` |

-----

### 2\. Command to Replace File Content

Navigate to the root of your workspace (e.g., your `colcon` or `catkin` workspace) and execute the `sed` command below. This command **recursively** replaces the old username with your new username across the contents of **all files**.

⚠️ **Warning:** This command performs **in-place editing**, modifying the files directly. Proceed with caution.

```bash
# General Syntax:
# Replace the placeholders (OLD_USER and NEW_USER) with your specific values.
find . -type f -exec sed -i 's/OLD_USER/NEW_USER/g' {} +
```

#### Example Implementation

If the old username is `marwan` and your new username is `robot-dev`:

```bash
find . -type f -exec sed -i 's/marwan/robot-dev/g' {} +
```

-----

### 3\. Verify Changes (Optional Check)

After running the replacement command, use `grep` to confirm if any instances of the original username remain in any files:

```bash
# Replace 'OLD_USER' with the old username you searched for (e.g., 'marwan')
grep -r 'OLD_USER' .
```

  * If this command returns **no output**, the replacement was successful. 🎉
  * If it returns file paths and lines of code, you may need to manually inspect and correct those remaining files.