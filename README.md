# HOWTO: Robonomics demo with Curiosity Rover moving after transaction and storing data in blockchain
Sample of how it works is available on YT: https://www.youtube.com/watch?v=6BSOyRbmac8 
### Requirements:
- ROS Melodic + Gazebo + RViz (installation manual [here](http://wiki.ros.org/melodic/Installation))
- extra packages:
```shell
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller
```
- IPFS up to [0.6.0](https://dist.ipfs.io/go-ipfs/v0.6.0/go-ipfs_v0.6.0_linux-386.tar.gz)
- Robonomics node (binary file) (download latest release [here](https://github.com/airalab/robonomics/releases). This tutorial tested fine on v1.1)

------------

### 1. Set up a simulation
Download Curiosity rover package:
```shell
mkdir -p robonomics_ws/src
cd robonomics_ws/src
git clone https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/
cd ..
catkin build
```
We need to adjust starting conditions to make our rover spawn smoothly:
- Go to

`src/master/curiosity_mars_rover_description/worlds` and change line 14 of the file` mars_curiosity.world` to 
`<pose>0 0 8 0 0 0</pose>`

- Go to

`src/master/curiosity_mars_rover_description/launch` and change line 4 of the file `mars_curiosity_world.launch` to 
`<arg name="paused" default="false"/>`

Don't forget to add source command to `~/.bashrc`
`source /home/$USER/robonomics_ws/devel/setup.bash`


- Reboot console and launch the simulation:

```shell
roslaunch curiosity_mars_rover_description main_real_mars.launch
```
![Mars rover](./media/rover.png?raw=true "Mars rover")

Note: if the image is dark, e.g. shadowed, change `Camera` to `Orthorgraphic` in Gazebo toolbar.
The simulation can be closed for a while.

------------

### 2. Download Robonomics controller package
To download a controller package for Rover type in terminal:
```shell
cd ~/robonomics_ws/src
git clone https://github.com/PaTara43/robonomics_sample_controller
cd robonomics_sample_controller
pip3 install -r requirements.txt
cd ..
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 # The controller supports python3
```

------------

### 3. Manage accounts in DAPP
Since we are testing, let us create a local robonomics network with robonomics binary file:
```shell
./robonomics --dev --tmp
```

![Running node](./media/robonomics.png?raw=true "Running node")


Go to https://parachain.robonomics.network and switch to local node 

![Local node](./media/local-node.png?raw=true "Local node")


Go to Accounts and create **CURIOSITY** and **EMPLOYER** accounts.

**Important**! Copy each account's address (to copy address click on account's icon) and Curiosity's account **mnemonic seed** (obtained while creating the account)
Transfer some money (units) to these accounts. You can read more about accounts in Robonomics [here](https://wiki.robonomics.network/docs/en/create-account-in-dapp/)

![Account creation](./media/account-creation.png?raw=true "Account creation")


Add these addresses, seed and node address (defaults to `ws://127.0.0.1:9944` for developer node) in `config.config` in `robonomics_ws/src/robonomics_sample_controller/src`. No quotes.

------------


### 4. Start Robonomics
In a separate terminal launch IPFS:
```shell
ifps init #you only need to do this once per IPFS installation
ipfs daemon
```

In another separate terminal launch Curiosity simulation if it's not live:
```shell
roslaunch curiosity_mars_rover_description main_real_mars.launch
```
Wait till it stays still

In another terminal launch the controller:
```shell
rosrun robonomics_sample_controller sample_controller.py
```
![Controller](./media/controller.png?raw=true "Controller")


Now you can send a transaction triggering the Rover to start moving and collecting data. To do so, you can use the same portal https://parachain.robonomics.network.
Go to `Developer->Extrinsics` and select Curiosity's employer account, `launch` extrinsic, Curiosity's account as a target account and `yes` as a parameter.
Submit the extrinsic.

![Extrinsic](./media/extrinsic.png?raw=true "Extrinsic")


The robot should start moving. It won't accept commands from other accounts neither commands with `no` parameter. The rover will move around and collect data for about a minute.
Later, when the job is done:

![Job done](./media/job-done.png?raw=true "Job done")


On the Robonomics portal go to `Developer -> Chain state` and obtain a `CURIOSITY` datalog using “+” button with selected `datalog -> RingBufferItem` as query: 

![Datalog](./media/datalog.png?raw=true "Datalog")

Now the IPFS hash of the telemetry is saved in the blockchain. To see the data simply copy the hash and find it on a gateway:

![Data in IPFS](./media/data-in-ipfs.png?raw=true "Data in IPFS")


This telemetry is kept in a decentralized storage, and it's hash is stored in a blockchain!

