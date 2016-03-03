# Working with the PythonWorkspace

## File Organization
- `idash.py` is a small class that provides an interactive plotting dashboard!
- `vrep.py`, `vrepConst.py`, and `remoteApi.so` are files necessary to get the remote vrep Python client to run
- `question1.py` is the remote api script that solves the problem for question 1 of the final project. Use similar names and structure for the other questions.
- `robot_helpers.py` give a class we can import into all question scripts that provide us with common functions to build a robot model that will sense and control it for general tasks used in all questions

## Running the code

Of course you must have vrep running. On way to get the remote API started when you boot vrep can be seen in `run_question1.sh` (please make different run scripts with similar structure for the remaining questions).  This file currently has my (Ryan's) hardcoded path.  Sorry about that!  I will run the `go.sh` command to get both the vrep server running and the client connecting, so the simulation runs without need for pressing any play buttons! Pretty untested for Windows!