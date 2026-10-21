# Buck-E repository working location

The user's primary checkout on this Windows computer is:

`C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone`

Use that checkout for future repository inspection, implementation, tests, commits, and pushes. Do not treat the older checkout under `Documents\ChatGPT\Buck-E Capstone\buck-e-capstone` or the `arm-preparation` staging folder as the primary project.

Update the older checkout only when the user asks to synchronize it. Preserve local changes in either checkout and inspect Git status before updating. On other machines, including the arm's Pi, use the actual local repository path.

The arm defaults to simulated status. Physical telemetry requires an explicit hardware backend selection. Current hardware support is read-only; adding or executing motor movement requires separately verified hardware/safety details and explicit user authorization. See `arm/docs/pi_verification.md` for confirmed observations and pending checks.
