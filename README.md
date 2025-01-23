## 2. Waypoint Navigation
This part implements a waypoint monitoring system for a drone using GPS data and real-time flight logic. The drone follows a series of waypoints and dynamically adjusts its path during the mission based on various conditions. The system includes functionality for adding, removing, and modifying waypoints during flight, ensuring adaptive mission control and monitor the mission


### Algorithm and Implementation
#### Waypoint Navigation
* **Sending the waypoints:**
Send the predefined waypoints to the drone for the navigation using mavlink, in this we used `pymavlink` package

#### Waypoint Monitoring

* **Estimated Distance and Time Calculation:** Get the distance to the next waypoint from the `NAV_CONTROLLER_OUTPUT` message and get the speed from the `VFR_HUD` message. calculate the distance between the next waypoint to the last waypoint using a function, them calcute the estimated time from them

* **Waypoint Number:** Get the current and last waypoint numbers and use them to know the status of the mission


#### Takeoff:
* The drone is programmed to take off autonomously to a defined altitude before initiating waypoint navigation. This ensures obstacle clearance and stability.
* The altitude for takeoff is specified in the mission parameters or as a default value in the code.

#### Landing:
* Once the final waypoint is reached, the drone executes an automated landing sequence.
* Landing behavior includes gradual descent and ensuring stability on touch-down.

#### Flight Path Visualization
* Plot the waypoints using the matplotlib
* Update the plot dynamically while the drone is moving


### How to Run the Code
For this navigation project we are using a simulated drone **SITL(Software In The Loop)** provided by Ardupilot, so first we need to download that software. Follow the steps in the Requirements section, now we are good to go

Open the terminal and run the below command, it will launch the SITL, map and a console to monitor the drone
```bash
sim_vehicle.py -v ArduCopter --console --map
```
we can control the drone using the [mavlink](https://mavlink.io/en/) commands from the terminal as show in the below command
```bash
mode GUIDED
```
above command set the drone to `GUIDED` mode, also we can send commands to follow waypoints and so on

Now let's run the code to follow our pre-defined(this repo is located in you `HOME` directory)
```bash
cd $HOME/vecros_assignment/drone_navigation
python3 drone_navigation.py
```
After running the command you will see the result

you can also run the drone in any shape like love symbol, go to `shape_generator.py` file and create waypoints using your desired function as shown below

```python3
waypoints = generate_love_symbol_waypoints(center_lat=-35.3635, center_lon=149.1641)
```

### Output
#### Drone navigation to waypoints
Drone navigating through the predefined waypoints and the path is drawn using matplotlib to know the drone status, also printed the estimated value in the terminal

![drone_navigation_video](drone_navigation/drone_nav.webm)


#### Changing the waypoints during the mission
After reaching the 10th waypoint, drone added a new waypoint perpendicular to the direction of the drone in 100m, then continue the mission
Before reaching the 10th point
![drone_navigation](drone_navigation/drone_navigation.png)
After reaching the 10th point
![drone_navigation_and_reroute](drone_navigation/drone_navigation_and_reroute.png)


#### Moving the drone in a shape
Generated some waypoints for moving in a desired shape using a function and make the drone to follow those waypoints
![love_symbol_shape](drone_navigation/love_symbol_shape.png)


### Conclusion
This project demonstrates the functionality of waypoint monitoring and dynamic path adjustment for drone missions. The ability to update waypoints in-flight and return to home ensures flexibility in real-world applications. The system can be further extended to include obstacle avoidance, mission planning, and integration with more advanced flight control algorithms.
