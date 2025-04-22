# CIEM6220 - Unit 1: Integration project

## Installation steps
Note: these installation steps will be largely based on Visual Studio Code as your main interface. Other IDEs can be used, but the creation / activation of appropriate conda environments will have to be handled manually.

- [ ] Download and install [Visual Studio Code](https://code.visualstudio.com/)
- [ ] Download and install [miniconda](https://www.anaconda.com/docs/getting-started/miniconda/main) (recommended) or your preferred version of anaconda, to allow creating project-based environments
- [ ] Download and install [SUMO](https://eclipse.dev/sumo/)
- [ ] Launch Visual Studio Code
- [ ] In Visual Studio Code, select File -> Open folder.. and navigate to the folder containing this repository.
- [ ] Open (any) one of the project's Jupyter notebooks (e.g. 'A20_simulation.ipynb')
- [ ] Click on "Select kernel..." on the top right of the Jupyter notebook screen, select "Python Environments..." and "Create new python environment". Create a ".conda" environment. Each time you open VSCode and this project, it will remember this is the right environment and always use it.
- [ ] Open VSCode's terminal (menu bar -> Terminal) *after* creating and selecting your .conda enviroment.
- [ ] Activate your new enviromnent by typing the following in the terminal window: conda activate .conda/
- [ ] Install *lxml* by typing the following in the terminal window: pip install lxml
- [ ] Install *tud-sumo* by typing the following in the terminal window: pip install tud-sumo==3.2.1
- [ ] Install *mercury* by typing the following in the terminal window: pip install mercury

## Running steps
- [ ] In the terminal window, type "mercury run". If everything went smoothly during installation, your browser will open with a new window. Have fun!


## FAQs:
Q: Which packages should I have in my conda enviroment to be sure everything works?
A: Please check the [requirements.txt](requirements.txt) file.

Q: When trying to run the notebook I get errors related to missing packages. What do I do?
A: Check carefully python's error messages. When a library / package is missing, python will let you know which one it is. Install missing packages via pip.

Q: I get weird errors when trying to run / visualize a simulation, something about SUMO_HOME not being set
A: Edit the '.env' file in your project's main folder. Add a line that defines where SUMO_HOME is (e.g.: SUMO_HOME=C:/SUMO/). Please note this is computer (and Operating System) dependent, SUMO will define its home folder during installation.

Q: tud-sumo seems to not work / I get strange errors immediately as I attempt to run this notebook.
A: tud-sumo is under constant development, at times this might mean bugs are not ironed out. Try installing the stable version, following the instructions below:
- [ ] Install *tud-sumo* by typing the following in the terminal window:
    - git clone -b stable https://github.com/DAIMoNDLab/tud-sumo.git
    - pip install ./tud-sumo