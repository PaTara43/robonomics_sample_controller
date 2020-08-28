<p style="color: red; font-weight: bold">>>>>>  gd2md-html alert:  ERRORs: 0; WARNINGs: 0; ALERTS: 11.</p>
<ul style="color: red; font-weight: bold"><li>See top comment block for details on ERRORs and WARNINGs. <li>In the converted Markdown or HTML, search for inline alerts that start with >>>>>  gd2md-html alert:  for specific instances that need correction.</ul>

<p style="color: red; font-weight: bold">Links to alert messages:</p><a href="#gdcalert1">alert1</a>
<a href="#gdcalert2">alert2</a>
<a href="#gdcalert3">alert3</a>
<a href="#gdcalert4">alert4</a>
<a href="#gdcalert5">alert5</a>
<a href="#gdcalert6">alert6</a>
<a href="#gdcalert7">alert7</a>
<a href="#gdcalert8">alert8</a>
<a href="#gdcalert9">alert9</a>
<a href="#gdcalert10">alert10</a>
<a href="#gdcalert11">alert11</a>

<p style="color: red; font-weight: bold">>>>>> PLEASE check and correct alert issues and delete this message and the inline alerts.<hr></p>


**HOWTO: Robonomics demo with Curiosity Rover moving after transaction and storing data in blockchain**


```
Requirements:
ROS Melodic + Gazebo + RViz (installation manual here)
extra packages:
$ sudo apt-get install ros-melodic-gazebo-ros-control 
$ sudo apt-get install ros-melodic-effort-controllers
$ sudo apt-get install ros-melodic-joint-state-controller
IPFS 0.4.22 (download from here and install)
ipfshttpclient:
$ pip install ipfshttpclient
Robonomics node (binary file) (download latest release here)
IPFS browser extension (not necessary)
```




1. **Set up a simulation**

Download Curiosity rover package:


    $ mkdir -p robonomics_ws/src


    $ cd robomonics_ws/src


    $ git clone [https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/](https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/)


    $ cd ..


    $ catkin build

We need to adjust starting conditions to make our rover spawn smoothly:



1. Go to

    /simulations_ws/src/master/curiosity_mars_rover_description/worlds and change line 14 of the file mars_curiosity.world to 


    &lt;pose>0 0 9 0 0 0&lt;/pose>

2. Go to

    /simulations_ws/src/master/curiosity_mars_rover_description/launch and change line 4 of the file mars_curiosity_world.launch to 


    &lt;arg name="paused" default="false"/>


Don’t forget to add source command to ~/.bashrc

	source /home/patara/robonomics_ws/devel/setup.bash \


Reboot console and launch the simulation:


    $ roslaunch curiosity_mars_rover_description main_real_mars.launch



<p id="gdcalert1" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image1.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert2">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image1.png "image_tooltip")


It can be closed for a while


    **2. Download controller package**



1. To download a controller package for Rover type in terminal:

    $ cd ~/robonomics_ws/src


    $ mkdir robonomics_sample_controller


    $ cd robonomics_sample_controller


    $ git clone [https://github.com/PaTara43/robonomics_sample_controller](https://github.com/PaTara43/robonomics_sample_controller)


    $ cd ../..


    $ catkin build


    **3. Manage accounts in DAPP**

1. Since we are testing, let’s create a local robonomics network with robonomics binary file:

    $ ./robonomics --dev --rpc-cors all


    ```
Important! After each launch it's necessary to remove a directory db in /home/$USER/.local/share/robonomics/chains/dev/
```



    

<p id="gdcalert2" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image2.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert3">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image2.png "image_tooltip")


2. Go to [https://parachain.robonomics.network](https://parachain.robonomics.network) and switch to local node 



<p id="gdcalert3" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image3.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert4">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image3.png "image_tooltip")




3. Go to Accounts and create CURIOSITY and EMPLOYER accounts (NOT_CURIOSITY is not necessary)

    ```
Important! Copy each account key and address (to copy address click on its icon)
```



	Transfer some money (units) to these accounts



<p id="gdcalert4" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image4.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert5">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image4.png "image_tooltip")




<p id="gdcalert5" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image5.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert6">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image5.png "image_tooltip")




4. Add these addresses and path to robonomics folder to file config.config in robonomics_ws/src/robonomics_sample_controller/src
1. **Start Robonomics**

Up to now the <span style="text-decoration:underline;">only thing running should be the robonomics local node</span>



1. In a separate terminal launch IPFS:

    $ ifps init (you only need to do this once)


    $ ipfs daemon

2. In another separate terminal launch Curiosity simulation:

    $ roslaunch curiosity_mars_rover_description main_real_mars.launch


Wait till it stays still



3. In another terminal launch the controller:

    $ roslaunch curiosity_mars_rover_description main_real_mars.launch




<p id="gdcalert6" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image6.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert7">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image6.png "image_tooltip")




4. Now you can send a transaction triggering the Rover to start moving and collecting data. To do so, you should use the [Robonomics IO](https://wiki.robonomics.network/docs/rio-overview/) subcommand of robonomics binary file:

    $ echo "ON" | ./robonomics io write launch -r &lt;CURIOSITY ADDRESS> -s &lt;EMPLOYER’S KEY>


    Where &lt;CURIOSITY ADDRESS>  and &lt;EMPLOYER’S KEY> are replaced with  previously saved strings respectively

5. You should see the following:	

<p id="gdcalert7" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image7.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert8">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image7.png "image_tooltip")

6. And the robot should start moving. Later, when the job is done:

    

<p id="gdcalert8" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image8.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert9">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image8.png "image_tooltip")



    On the Robonomics portal go to Developer -> Chain state and add a CURIOSITY datalog using “+” button with selected “datalog” as state query: 

<p id="gdcalert9" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image9.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert10">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image9.png "image_tooltip")


7. Now the IPFS hash of the telemetry is saved in the blockchain. To see the data simply copy the hash and insert it in IPFS Companion:

    

<p id="gdcalert10" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image10.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert11">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image10.png "image_tooltip")


8. Click Explore -> View on Gateway and voila!

    

<p id="gdcalert11" ><span style="color: red; font-weight: bold">>>>>>  gd2md-html alert: inline image link here (to images/image11.png). Store image on your image server and adjust path/filename/extension if necessary. </span><br>(<a href="#">Back to top</a>)(<a href="#gdcalert12">Next alert</a>)<br><span style="color: red; font-weight: bold">>>>>> </span></p>


![alt_text](images/image11.png "image_tooltip")
