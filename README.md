# Build Repository for [B2EMO RC Toy][thangs_upstream] by [Boxandloop][boxandloop]

This repository contains my notes and any modifications made while building Boxandloop's B2EMO RC Toy.

Status:
- waiting on electronics/hardware parts to arrive
- slicer layout prepped, will start printing soon
- started reviewing and modifying the provided arduino code,
  while learning about arduino for the first time

Basic import and cleanup of the [vehicle](vehicle/) and [remote](remote/) code from upstream pdf fixes several problems:
- smart quotes in the vehicle code
- lots of indendtation / nesting problems, but those may have been due to bad text flow out of my PDF reader's export/copy?
- added `sketch.yaml` definitions to make development more reproducible

Further refactoring and improvement in [dev branch](../../commits/dev/):
- **warning** untested, as I don't yet have hardware
- **warning** this is my first arduino project, so I'm mostly learning by refactoring,
  and applying pattern matching from other domains of programming where I'm more familiar

NOTE: unclear what the license is for the upstream project, so:
- not adding STL files to this repository (yet?)
- nor adding my 3mf slicer file (yet?)

[boxandloop]: https://thangs.com/designer/Boxandloop
[thangs_upstream]: https://thangs.com/designer/Boxandloop/3d-model/B2EMO%20RC%20Toy-1032786
