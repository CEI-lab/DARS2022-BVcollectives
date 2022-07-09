# Braitenberg Vehicles

This repository contains code for simulating and analyzing Braitenberg vehicles
with on-board stimuli sources.

# Requirements

- Python 3
- Matplotlib 3.5+ (for plots)
- [PyGame](https://www.pygame.org/news) (installable with pip)

# Usage

Run `multiple_sweeps.py` to generate the main figures in the paper (uncomment
sections as desired).

Run `demo_interactive.py` to interact with a collective using a single
controllable very bright agent.

Run `generate_demo.py` and then `play_demo.py` after changing relevant settings
at the top of the files to generate a video of a single run.

Configuration files are in yaml format, in the `configs/` folder.

Other files contain functionality that may be useful for running different types
of simulations or improving the simulator:
- `robots.py` contains the agent model implementation
- `utils.py` contains geometric and other helper functions
- `param_sweep.py` contains functions for running parameter sweeps
- `unit_test.py` contains tests for the simulator
