# Welcome to the `Rogatka` Module!!
##  Sections of this readme
* Simulation
    * Installation & Configuration
    * Running
    * Connecting to the Simulation
        * With MissionPlanner
        * With a Script
* Navigation
    * To Be Continued...

## Simulation
During this section, make sure you run all the referenced scripts directly from the `Rogatka` folder.  
### Installation & Configuration
1. Run the `simulation_installation.sh` file. It can take up to an hour to run.
2. Restart the WSL:
    1. Close the WSL.
    2. Open windows powershell.
    3. Run the `wsl --shutdown` command.
    4. Open the WSL again.
3. Run the `simulation_config.sh` file. It can take up to fifteen minutes to run.

Congratulations! You have successfully installed the drone simulator.

### Running
After you finished the installation steps detailed above, running the simulation is really simple. We made it even simpler - you just need to run the `run_simulation.sh` file, and the simulation will start running.

It has many cool options, for example under `Tools > Servo Outputs` you can see in real time the output to all four servoes.

### Connecting to the Simulation
We will start this section with a quick review of the simulation framework.  
The simulation you just installed runs a Software In The Loop (SITL) model. This is a model that allows connecting to the simulation using a script over the network.

By default, the SITL listens on the `localhost` address, a.k.a `127.0.0.1`, on 3 ports: number `5760`, number `5762` and number `5763`. It uses internally the `5760` port, and we will use the other two for our purposes.

#### With MissionPlanner
First, install MissionPlanner (on windows) from [this link](https://ardupilot.org/planner/docs/mission-planner-installation.html).

After you finish the installation, open the app. On the top left corner you will see a `connect` button. Press it, and in the popups make sure you connect to the `127.0.0.1` address with port number `5762`.

Wait for it to load. On the scren you can see the map with the drone on it (zoom in if you need to). on the left side, you can see the drone telemetry - altetuide, ground speed and more. We will use this screen for easy monitoring of the simulation.

### With a Script
Once the simulation is running, connecting to it with a python script is very easy.

You need to install the `dronekit` python module, using `pip3 install dronekit`.  
After that, whenever you want to connect a script to the simulation, simply start it with the following code snippet:
```
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import ... # Replace with relevent classes and functions

connection_string = "tcp:127.0.0.1:5763"

# Connect to the Vehicle
print("Connecting to vehicle on: {}".format(connection_string))             vehicle = connect(connection_string, wait_ready=True)
```

This will connect to the remaining open port in the simulation and setup the simulated drone for use. To write the rest of the code, use the official [dronekit documentation](https://dronekit.netlify.app/).

