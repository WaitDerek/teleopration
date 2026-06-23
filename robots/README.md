# Robot Backends

Robot-specific control packages live here.

`ur10e_rtde/` is implemented in this workspace as a submodule-ready package. When the remote repository exists, convert it to a real submodule mounted at the same path:

```bash
git submodule add <ur10e-rtde-repo-url> robots/ur10e_rtde
```

The root teleoperation package must not import robot backends by default.
