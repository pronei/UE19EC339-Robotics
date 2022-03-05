# Robotic Systems - UE19EC339
Repository that contains the starter codes for assignments and general instructions, links and resources.

## Project I
The `cube_sim` package contains the necessary starter code and a utility class that implements a discrete integrator.

The object of this project is to understand and develop a framework for simulating an object, in our case, a simple cube in 3
dimensions. Whenever a force or a torque is applied to an object, its motion and pose is described by
[Newton's laws](https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion) and the extended 
[Euler's Laws](https://en.wikipedia.org/wiki/Euler%27s_laws_of_motion) of motion.

![gif of cube moving](demo_cubesim.gif)

### How to run
1. Clone this repo and `cd` into `cube_sim`. This is the Python package with all the necessary scripts.
1. To view the demo, run `3d_cube_anim.py` with Python3.
1. Once you have completed imeplementing `simulator.py` and uncommented the neccessary lines in `3d_cube_anim.py`,
run `3d_cube_anim.py` again.

### Implementation details in `simulator.py`
1. If you're going to use an integrator, initialize it accordingly in the constructor for `MySolver`.
1. The solver's state maintained at each iteration consists of the state vector and its first derivative,
the array of force vectors and an iteration number. The state vector is modelled as a 6x1 vector <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}_{cube} = [\ x\quad y\quad z\quad \phi\quad \theta\quad \psi\ ]^\top">
and <img src="https://render.githubusercontent.com/render/math?math=\dot{\mathbf{x}}_{cube} = [\ \dot{x}\quad \dot{y}\quad \dot{z}\quad \dot{\phi}\quad \dot{\theta}\quad \dot{\psi}\ ]^\top">. Each force vector is modelled as <img src="https://render.githubusercontent.com/render/math?math=\mathbf{F}_{cube} = [\ F_x\quad F_y\quad F_z\quad \tau_x\quad \tau_y\quad \tau_z\ ]^\top">.
1. Each successive call to `next_iter` should update the state. Don't return the state in this method, use `get_x` for that.
1. The getters should return the state and its derivative.
1. 


## Mini-assignment I
The package is contained in `action_tutorial`.

The objective of this assignment is to understand how [**actions**](https://design.ros2.org/articles/actions.html) work in ROS.
![](https://docs.ros.org/en/foxy/_images/Action-SingleActionClient.gif)

### Overview
As seen from the above illustration, the package serves to replicate the behavior of **actions** in ROS using the example of a simple
counter. There are 2 main services `GoalService` and `ResultService` alongside a single pub-sub on `/feedback_topic`. The client sends a 
`goal` request to the server containing the value to be counted to, assuming that the count starts from 0. To
this, the server responds back with an acknowledgement and begins the process of counting. A timer based callback is then setup on the server to increment the counter at intervals of 1 second.
To initiate feedback from the server, the client sends another `result` request to the server and waits for a response to the same (this is sent when the counter's final value is reached).
Once this service call is handled on the server, it begins to publish "feedback" on `/feedback_topic`, which is the current value of the
counter at that instant of time. The client having subscribed to this feedback topic, listens to it and logs the data on it.
Once the counter's end value is reached, the server responses to the `result` request sent earlier. Both client and server log the final
values of the counter and continue to spin.

### How to run
1. Clone this repo, copy the `action_tutorial` folder into your workspace/src and build using `catkin_make`. 
2. Add `source ~/<path_to_your_ws>/devel/setup.bash` to the end of your `~/.bashrc`.
3. Start `roscore` in a new terminal.
4. In a new terminal, run `rosrun action_tutorial counter_action_server.py`.
5. In another terminal, run `rosrun action_tutorial counter_action_client.py <counter_end_value>`.

### Functionality to be implemented
1. A starting value to the counter instead of 0 (default).
2. A rate (in Hz) to the timer based callback for incrementing.
3. **[BONUS]** The server should decline `GoalService` in cases where the start value exceeds the end of the counter at the time of making the request from the client's side.

### Guidelines
- Follow the TODOs mentioned inside [`counter_action_client.py`](action_tutorial/scripts/counter_action_client.py), [`counter_action_server.py`](action_tutorial/scripts/counter_action_server.py) and [`CounterGoal.srv`](action_tutorial/srv/CounterGoal.srv).
- Upon completing each task mentioned under a TODO, please delete the corresponding TODO comment.
- An automated script will grade your code depending on the functionality achieved so please test your code before submitting.
- Submit a zip of the modified package folder `action_tutorial`. The zip should bear your name and SRN (`Firstname_PES1UG19ECXXX`).

## Environment setup for group based projects

### Prerequisites
1. Ubuntu 18 or Ubuntu 20 (VM of the same alternatively if not possible to install)
   - 8GB (or above) pendrive for Ubuntu Live USB (for installing Ubuntu itself)
   - 16GB (or above) pendrive for Ubuntu
2. ROS Melodic (Ubuntu 18) or ROS Noetic (Ubuntu 20)
3. Gazebo 11

### Useful links
- [Tutorial](https://www.fosslinux.com/10212/how-to-install-a-complete-ubuntu-on-a-usb-flash-drive.htm) for installing Ubuntu 20 on pendrive
- [Tutorial](https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/) for dual booting Ubuntu with Windows **(TAs are not responsible in case of any data loss or OS wipe)**. Please follow instructions carefully if you are going this route.
- Installation of [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) for Ubuntu 18
- Installation of [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu 20
- Installation of [Gazebo](http://www.gazebosim.org/tutorials?tut=install_ubuntu) for Ubuntu 18/20

## Resources
- Official [ROS wiki](wiki.ros.org). Contains a conceptual overview as well as step by step tutorials to help
you get a feel for ROS and its capabilities.

