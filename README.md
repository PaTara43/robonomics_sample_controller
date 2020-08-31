# HOWTO: Robonomics demo with Curiosity Rover moving after transaction and storing data in blockchain
Sample of how it works is available on YT: https://youtu.be/pl3eIEC_T2o
### Requirements:
- ROS Melodic + Gazebo + RViz (installation manual [here](http://wiki.ros.org/melodic/Installation))
- extra packages:
```shell
sudo apt-get install ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joint-state-controller
```
- IPFS 0.4.22 (download from [here](https://dist.ipfs.io/go-ipfs/v0.4.22/go-ipfs_v0.4.22_linux-386.tar.gz) and install)
- ipfshttpclient:
```shell
pip install ipfshttpclient
```
- Robonomics node (binary file) (download latest release [here](https://github.com/airalab/robonomics/releases))
- IPFS browser extension (not necessary)

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

`/robonomics_ws/src/master/curiosity_mars_rover_description/worlds` and change line 14 of the file` mars_curiosity.world` to 
`<pose>0 0 9 0 0 0</pose>`

- Go to

`/robonomics_ws/src/master/curiosity_mars_rover_description/launch` and change line 4 of the file `mars_curiosity_world.launch` to 
`<arg name="paused" default="false"/>`

Dont forget to add source command to `~/.bashrc`
`source /home/$USER/robonomics_ws/devel/setup.bash`


- Reboot console and launch the simulation:

```shell
roslaunch curiosity_mars_rover_description main_real_mars.launch
```
![Curiosity](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2017-22-28.png?raw=true "Curiosity")

It can be closed for a while

------------

### 2. Download controller package
To download a controller package for Rover type in terminal:
```shell
cd ~/robonomics_ws/src
mkdir robonomics_sample_controller
cd robonomics_sample_controller
git clone https://github.com/PaTara43/robonomics_sample_controller
cd ../..
catkin build
```

------------

### 3. Manage accounts in DAPP
Since we are testing, let us create a local robonomics network with robonomics binary file:
```shell
./robonomics --dev --rpc-cors all
```


**Important!** Before next launches it is necessary to remove a directory `db` with

`rm -rf /home/$USER/.local/share/robonomics/chains/dev/db`



![Robonomics node](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2018-21-52.png?raw=true "Robonomics node")

Go to https://parachain.robonomics.network and switch to local node 

![Local node](https://wiki.robonomics.network/assets/static/robonomics-dapp-connect-local.09c0af9.8bb6632a8836118ad6b6049d0852c1eb.jpg "Local node")

Go to Accounts and create **CURIOSITY** and **EMPLOYER** accounts (**NOT_CURIOSITY** is **not** necessary)

**Important**! Copy each account's key and address (to copy address click on account's icon)
Transfer some money (units) to these accounts

![Add Account](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2018-27-47.png?raw=true "Add Account")

![Balances](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2018-33-14.png?raw=true "Balances")

Add these addresses and path to robonomics folder to file `config.config` in `robonomics_ws/src/robonomics_sample_controller/src`

------------


### 4. Start Robonomics
Up to now the **only thing running** should be the robonomics local node
In a separate terminal launch IPFS:
```shell
ifps init #you only need to do this once
ipfs daemon
```

In another separate terminal launch Curiosity simulation:
```shell
roslaunch curiosity_mars_rover_description main_real_mars.launch
```
Wait till it stays still

In another terminal launch the controller:
```shell
rosrun robonomics_sample_controller sample_controller.py
```
![Running controller](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2018-46-30.png?raw=true "Running controller")

Now you can send a transaction triggering the Rover to start moving and collecting data. To do so, you should use the [Robonomics IO](https://wiki.robonomics.network/docs/rio-overview/) subcommand of robonomics binary file:
```shell
echo "ON" | ./robonomics io write launch -r <CURIOSITY ADDRESS> -s <EMPLOYER’S KEY>
```
Where `<CURIOSITY ADDRESS>`  and `<EMPLOYER’S KEY>` are replaced with  previously saved strings accordingly

You should see the following:

![Arming](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2018-58-36.png?raw=true "Arming")

And the robot should start moving. Later, when the job is done:

![Done](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2019-10-43.png?raw=true "Done")

On the Robonomics portal go to Developer -> Chain state and add a CURIOSITY datalog using “+” button with selected “datalog” as state query: 

![Datalog](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2019-16-49.png?raw=true "Datalog")

Now the IPFS hash of the telemetry is saved in the blockchain. To see the data simply copy the hash and insert it in IPFS Companion:

![IPFS](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2019-18-58.png?raw=true "IPFS")

Click Explore -> View on Gateway and voila!

![Voila](https://github.com/PaTara43/media/blob/master/Screenshot%20from%202020-08-27%2019-20-01.png?raw=true "Voila")


