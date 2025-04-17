# CIEM6220 - Unit 1: Integration project

## Introduction

This repository contains everything required to complete the integration project for course CIEM6220, Unit 1.
Please refer to course materials (Brightspace) to align with your duties.

This software builds on [the mercury project](https://runmercury.com). The typical python notebook is turned therefore in a simple, easy-to-use Graphical User Interface.

## Getting started

- [ ] Please check [the install guide](INSTALL.md) first. You will need some software on your computer before you can run this notebook.
- [ ] (RECOMMENDED) Please complete *the Mercury tutorial notebook* first. This will help you familiarise with what can be done with this system

## Handling your integration project

There is no fixed order or set of instructions as to which tests to perform first in relation to your integration project. However, we compiled the following table to help you plan your actions in relation to when certain prerequisite knowledge is available (and therefore when a certain activity can be started for U1)

| Week number | U2-U1 prerequisites       | U3-U1 prerequisites         | U4-U1 prerequisites                 | Suggested U1 activities             |
| ----------- | ------------------------- | --------------------------- | ----------------------------------- | ----------------------------------- |
| 1           | X                         | X                           | Multi-lane traffic, bottlenecks     | Installation, configuration, base scenario analysis (where are the bottlenecks? what happens?) |
| 2           | X                         | IV control logic, ACC       | Variable Speed Limits               | Comparison between base scenario and managed (VSL only) scenario; impact of AV pen. rates |
| 3           | X                         | IV control logic            | X                                   | Selection of CACC model parameters, tuning, investigation of impacts |
| 4           | X                         | IVs and Beh. Adaptation     | Car-following models                | Selection of human vehicle model parameters, tuning, investigation of impacts. Literature on Beh. adaptation | 
| 5           | Surrogate Safety Measures | IVs and Traffic flow impact | Local and coordinated ramp metering | Analysis of SSMs, comparison of evolution of SSMs for different scenarios | 
| 6           | AVs and human safety      | IVs and Traffic flow impact | X                                   | Further analysis, final scenario set design and experimental setup |
| 7           | X                         | X                           | X                                   | Experiments, analysis, preparation for final presentation |
| 8           | X                         | X                           | X                                   | Analysis, preparation for final presentation |
| 9           | X                         | X                           | X                                   | X | 

## Getting support

During the quarter, classes for Unit 1 are held on Wednesdays 13:45-15:30. Otherwise, please reach out to us via Brightspace.

## FAQ & Known issues

Q: One (or more) of the simulation outputs is empty.
A: This can happen if the simulation does not generate the required output (e.g. if there are no active controllers the simulation will not have any related output).

Q: Something went wrong and the notebook is all crashed/broken now. What do I do?
A: Just refresh your browser's webpage, this effectively *resets the entire simulation scenario* and puts you back in a 'fresh' state.

Q: Simulating every scenario one-by-one takes too long. Is there a way to see results faster?
A: We developed a solution caching mechanism to help with quickly analysing different penetration rates and controllers (with, however, given threshold values). You can populate the solutions cache (once!) by running the 'populate_cache.py' file. This runs 44 simulations in batch and stores all results. This may take upwards of 30 minutes to complete.

## Authors and acknowledgment
This project was developed by Dr. Rinaldi in cooperation with the CIEM6220 2025 teaching team.

