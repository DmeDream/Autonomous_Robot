# Git snapshot of the autonomous robot project

This repository contains the Python mapping and route scripts, offline RRT*
planner, CoppeliaSim controller adapter, documentation, world generator/SDF,
map probabilities and metadata, pose arrays, planning results and screenshots.

Raw LiDAR recordings in `maps/*/data.csv` are stored as lossless `data.csv.gz`
archives next to their original metadata. Some raw CSVs exceed GitHub's normal
per-file limit. The original uncompressed files remain on the source machine.
`git_data_manifest.json` records each original file's byte count and SHA-256.

After cloning, restore the raw recordings when needed:

```bash
gzip -dk maps/*/data.csv.gz
```

The `-k` flag keeps each compressed archive. Run this after cloning, where the
uncompressed recordings are absent. Existing recordings do not need restoring.
The offline RRT* planner reads `ocgm_660610823.csv`, not the raw scan logs, so it
can run without extracting them.

Python caches, local virtual environments, generated runtime robot files,
process/session state and local credential files are excluded from Git.
Numeric CSV/NumPy files and compressed/image artifacts are tracked as binary
snapshots so Git does not normalize their line endings or merge numeric rows.

The ROS workspace used by this project is a separate directory:
`/home/d-dream/robot_ws`. Its ROS packages and installation are not part of this
folder snapshot. The current world source is included in `world_design/`; use
the setup instructions in `README.md` with the corresponding robot workspace.

This snapshot is taken before extending the mapping route to improve visibility
behind cabinets. Completing the existing 32-pose route does not establish full
visibility of occluded space. Measured unknown cells have been preserved.
