# Working with the PythonWorkspace

## Dependencies
In order to run the Python code, you will need the following libraries:

- numpy
- matplotlib
- scipy

## Running the code locally

#### Stage 1: Defend Goal during Penalty Kick
1. Open `Project_PlayingField_Q1.ttt` in V-REP.
2. Press Play to start the simulation and then Pause it
3. Run the final goalie strategy by using the command `python goalie2.py`. The simulation will be started automatically within the Python remote API script.

#### Stage 2: Dribbling
1. Open `Project_PlayingField_Q2.ttt` in V-REP.
2. Press Play to start the simulation and spawn the ball in a random place on the field.  If the ball is too close to the field boundary, stop the simulation and try again until it spawns closer towards center.
3. Run the dribbling strategy by using the command `python dribbling2.py`

#### Stage 2: Zone Passing
1. `Project_PlayingField_Q2.ttt` should have already been open, and the simulation running, by following the steps in __Stage 2: Dribbling__.
2. When the dribbling player enters _Zone 4_, quickly stop the dribbling code by pressing `Ctrl+C` or similar interrupt command for your system environment.
3. Quickly start the zone passing code while the simulator is still running, by typing `python zone_passer.py`.

#### Stage 3: Competition
1. Open `Project_PlayingField_Q3.ttt` in V-REP.
2. Press Play to start the simulation
3. To start the team, use the command `python matchplay.py Blue` to play as the `Blue` team; `python matchplay.py Red` to play as the `Red` team.  You can run these two commands in two different terminals to have `Blue` and `Red` play against each other.

## File Organization
- `goalie2.py` provides the answer for __Stage 1: Defending Goal during Penalty Kick__
- `dribbling2.py` provides the answer for __Stage 2: Dribbling__
- `zone_passer.py` provides the answer for __Stage 2: Zone Passing__
- `matchplay.py` provies the answer for __Stage 3: Competition__
- `robot_helpers.py` give a class we can import into all question scripts that provide us with common functions to build a robot model that will sense and control it for general tasks used in all questions
- `base_robot.py` contains the base classes for the player which other classes in other files will inherit.
- `idash.py` is a small class that provides an interactive plotting dashboard!
- `vrep.py`, `vrepConst.py`, and `remoteApi.so` are files necessary to get the remote vrep Python client to run
