# Buck-E repository working location

The user's primary checkout on this Windows computer is:

`C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone`

Use that checkout for future repository inspection, implementation, tests, commits, and pushes. Do not treat the older checkout under `Documents\ChatGPT\Buck-E Capstone\buck-e-capstone` or the `arm-preparation` staging folder as the primary project.

Update the older checkout only when the user asks to synchronize it. Preserve local changes in either checkout and inspect Git status before updating. On other machines, including the arm's Pi, use the actual local repository path.

`bucke_arm.arm` contains `RealArm` and `FakeArm` with matching application methods; no separate read-only interface or factory is used. Packaged status defaults to FakeArm; hardware use requires explicit RealArm construction or backend selection. `joint_control.py` is a small manual testing menu; keep reusable motion checks in RealArm rather than duplicating them in the menu. The user confirmed both saved standby/ready transitions work safely on October 28, 2026. Retain their starting-state, limit and completion checks in RealArm and local confirmation in the menu. New movement routes or hardware changes require verified details and user authorization. See `arm/docs/pi_verification.md` for observations and scope.
