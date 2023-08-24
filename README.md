# Running kuka tests with Crocoddyl Instructions

## Environment Setup (Ubuntu, the gepetto-gui is not compatible with Mac)
I couldn't get the setup to work correctly using venv, using conda appears to be a lot easier, because there are non-python system packages which need to be installed to run 
the gepetto viewer, and maybe for other parts of this as well.

Install the dependencies mentioned in the Gepetto readme: https://github.com/Gepetto/gepetto-viewer

```
sudo apt install openscenegraph
sudo apt install qtbase5-dev qt5-qmake
```

Then install the viewer itself:
```
conda install gepetto-viewer gepetto-viewer-corba -c conda-forge
```

I ended up installing pinocchio directly after wrestling with it for a while, I think crocoddyl should install this for you though:
```
conda install pinocchio -c conda-forge
```

But then installing from Conda should work:
```
conda install crocoddyl -c conda-forge
```

Run the arm_manipulation script in the examples/notebooks directory to make sure the installation worked correctly.

We also need the ![robot-descriptions]() package to get a pinocchio-compatible description of the IIWA robot, this one isn't included as part of Crocoddyl. So install that with Conda also:

```
conda install -c conda-forge robot_descriptions
```

## Running the test
In one terminal, run the gepetto-gui. Assuming it was installed correctly:

```
gepetto-gui
```

There often seem to be errors and warnings of the forms:

```
libpng warning: iCCP: known incorrect sRGB profile
```

and

```
Error reading file /home/a2rlab/anaconda3/envs/crocoddyl/share/gepetto-viewer/fonts/arial.ttfQOpenGLContext::swapBuffers() called with non-exposed window, behavior is undefined
```

Neither of these seem to matter, the visualization can still run properly.

Once that is running, execute the `test_iiwa.py` script to solve a trajectory optimization problem using the Crocoddyl DDP solver.