# Mitsubishi ROS2 Driver
Integration of ROS2 for Mitsubishi 6DOF robots, supporting Melfa RV-FR Series robots.<br />
This driver is provided by [Adiro.com](https://www.adiro.com/en/) in collaboration with [Software-byTS](https://www.software-byts.de/)<br />
<p align="center">
<img src="https://github.com/user-attachments/assets/619bedc0-90f8-4aa6-8775-58d68f43b7cd" width="600" height="283">
</p>

> [!IMPORTANT]
>At the moment just two models, RV-2FR-D and RV-4FRL-D, are supported by this driver. 
>The driver is tested and executable with the Humble distribution from ROS2 and MoveIt2. We recommend you use the same setup. 

# Installing and building procedure
You can either integrate the driver to your natively installed ROS/Moveit workspace or use our provided docker container to test it with your robot and get familiar with it.<br />
If you are using the driver natively, you have to figure out the IP-Address of your Robot (**ROBOT-IP**) and Host computer (**HOST-IP**). Change these values manually in the code of the driver. Now you are also able to control your real robot.<br />
You can also delete all docker relevant files if you are running the driver natively on your system.<br/> 
In case you choose our docker container, the following guide will help you to get started. 

## Installation/Setup on Windows os
First download and install the latest version of [Docker-Desktop](https://docs.docker.com/desktop/install/windows-install/). You maybe need to check the [installation guide](https://docs.docker.com/desktop/install/windows-install/) provided by Docker Inc. or admin rights for your PC to use it.<br />   
After that, open your preferred text editor or IDE (we will be using Visual Studio Code for the next steps) and clone the repository.<br /> 
The next step is to type in the IP of your robot and your host IP. For that you need to open the file **docker-compose.yml** and modify the environment variables **ROBOT_IP** (Line 39) and **HOST_IP** (Line 42).<br />
<br />
All done, now we can start our docker container. **Don't forget to run the Docker-Desktop app. in the background.** <br />
Now, you can either use a docker extension, the provided PowerShell script (**start.ps1**) or type in this command in the terminal: 
```js
docker compose up -d
```
Dockers is now starting to build the containers, this may take approximately 10 to 20 minutes. Your docker container should now be running. The next step is to open up the UI from our container. <br />
Open this [Page with NoVNC](http://localhost:8080/vnc.html?autoconnect=1&resize=scale) and you should see a empty linux UI.
Next, we open up the bash inside the container. Again, you have a couple of options to do this, e.g. try the PowerShell script (**attach.ps1**) or type in this command:
```js
docker exec -it ros2-mitsubishi-moveit bash
```
<br />

![Docker-Open-Bash](https://github.com/Adiro-Automatisierungstechnik-GmbH/Mitsubishi_ROS2_Driver/assets/168413005/41b25461-13a4-413e-be6d-e165aab26bd2)

<br />

## Simulate/Operate your robot
Inside the bash-shell, you can now start to simulate or connect to your real robot.<br />
But before that, it's important that you add the host- and robot-IP! 
And remember, if you are changing files you have to build the workspace in the container again:
```js
colcon build
```

### Simulation
To start up your Mitsubishi robot in Gazebo and RViz, type in the following shell command:
```js
ros2 launch mitsubishi_ros2_gazebo robot_sim_moveit.launch.py robot:=rv_2frb_d
```
This command starts up the Melfa RV-2FR-D robot, if you want to use the RV-4FRL-D change the last parameter like this -> 
**robot:=rv_4frl_d**.<br /><br />
![Docker-Open-Sim](https://github.com/Adiro-Automatisierungstechnik-GmbH/Mitsubishi_ROS2_Driver/assets/168413005/168e38af-7c87-442d-aaa4-14e64f73f24f)<br />

You can now start to test the robot in the simulation.<br /><br />
![Docker-Play](https://github.com/Adiro-Automatisierungstechnik-GmbH/Mitsubishi_ROS2_Driver/assets/168413005/2452574f-956d-46e1-8254-3cb28c5787ec)

### Controlling your robot

> [!WARNING]
> Make sure you operate the robot in a save and collsion-free environment!

To control your robot directly e.g. with RViz, must to establish a connection to the robot controller.
First, load the program REAL out of the MELFA_Program folder onto your robot controller. **Change the IP address in line 28** and start the program. 
The robot will drive in a start position and wait for new input.
You are now able to control your robot controller via ROS. You should also see the symbol of a running program on your Teaching Box, e.g. the R56TB: <br /><br />

![Docker-TeachingBox](https://github.com/Adiro-Automatisierungstechnik-GmbH/Mitsubishi_ROS2_Driver/assets/168413005/a34bd8f0-bdc7-43c5-a15d-2d6c685a2e7b)

<br />
We test the connection by starting RViz again and adding the robot IP as a parameter:

```js
ros2 launch rv_2frb_d_moveit_config robot.launch.py robot_ip:=YOUR-IP... 
```

You should now be able to see the robot in RViz. Remember to set the velocity and acceleration appropriate to the environment of your robot!
Now, you can start testing to move your robot in RViz or write your own nodes/programs.
<br /><br />

![Docker-Real-Movement](https://github.com/Adiro-Automatisierungstechnik-GmbH/Mitsubishi_ROS2_Driver/assets/168413005/eacf806c-d68b-412b-80ba-7e8b199bd811)


### Terminate the process

You can simply type in **exit** to get out of the bash shell and the container, and to stop the docker container you type in this command:
```js
docker compose down
```

> [!IMPORTANT]
> Make sure to shut down the docker container properly to prevent any problems with Docker-Desktop.

Every container should now be stopped and you can close docker desktop as well.
