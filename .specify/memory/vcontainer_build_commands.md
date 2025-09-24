# vcontainer build commands (Ubuntu)

Saved on: 2025-09-24

Preference: When the user is working inside their vcontainer on Ubuntu, only provide the following commands for configuring and building the example project (copy-and-paste ready):

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build (or skip if you only want to configure)
cmake --build . -- -j 1
```

Notes:
- These commands assume you are in the project's `build` directory (e.g., `cd /workspace/.../example_repo/.../build`).
- If you want me to run them from here, tell me explicitly and I will execute them in the saved terminal context.
- To change this preference, update or delete this file.
