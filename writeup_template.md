## Project: 3D Motion Planning
![Quad Image](./misc/Viewofcity.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
Please see below for all my comments on my project.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The Motion_planning.py file is based on teh backyard flyier program from the last project but with some important additions.
It has a new state Planning where the waypint planning happens. This includes setting up the goal_start at the center of the grid and goal_end variables, loading the colliders.csv file data to determine the grid and its obstacles, perfoming an a_star search to find a path and then sending the waypoints to be graphed using the msgpack package. Additionally, in order to achieve all these steps it uses a planning_utils.py file to store some of the funcitons that it uses like the Creation of the Grid, the enumeration of actions and the definition of valid actions as well as the implementation of the A_Star algorithm as well as the heuristic needed to calculate the path.

Some examples of the most important coded in my view:

    a. Addition of new state: PLANNING = auto() - It uses the auto() function whihch replaces it with an apprpiate number instead of pre-assigning a number.  
    b. Initialization of the plan_path funciton where the actual path planning is done:
```python
    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE 
        
```
    b.  Loading of the obstacle map and creation of the grid
```python
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
```
### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

a. I opend the colliders file. Read the first line and stored the left part into lhs and right part into rhs.
```python
with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                lhs, rhs = row[0],row[1]
                break
```
b. I split the lhs and rhs variables to only get the numerical value as I didn't need the labels.
```python       
        _, lat = lhs.split()
       
        _, lon = rhs.split()
        lat0, lon0 = float(lat),float(lon)
        
```
c. Set the home position to those values form above.
```python
    
        self.set_home_position(lon0, lat0, 0)
```

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/HomeLocation.png)

#### 2. Set your current local position
Below is how I retrieve my local positio to then use it to calulate my start point:
```python
        global_position = self.global_position
    
        local_position = global_to_local(global_position, self.global_home)
 ```       

#### 3. Set grid start position from local position
Using the north and east offset that is returned from the grid creation I calulate the grid start.

```python
grid_start = (abs(int(local_position[1])) - north_offset,abs(int(local_position[0])) - east_offset)
```
#### 4. Set grid goal position from geodetic coords
Here I created a helper function in planning_utils that received a lattitude, long and the global_home and it returns the a local position foe the goal. 
```python
def lat_lon_goal(goal_lat, goal_lon, in_global_home):
    #Receives the goal longitude, latitude and the global_home locaiton and returns the local 
    # goal position
    latlonposiion = [goal_lon, goal_lat, 0]
    local_y, local_x, _ = global_to_local(latlonposiion, in_global_home)
    return local_y, local_x
```
Then I add the offsets to approiately frame it on our map.
```python
goal_y, goal_x = lat_lon_goal(37.796391, -122.398300, self.global_home)
      
    
        grid_goal = (int(goal_y - north_offset), int(goal_x - east_offset))
```
The I used the medail axis technique to re calculate the start and end goal to make sure they are within a free space in the grid adding the helper function from one our our classes:
```python
def find_start_goal(skel, start, goal):
    # Medial Axis method to find a goal_start and goal_end that is in free space
    # from lessons
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells),                                    axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells),                                    axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    return near_start, near_goal
```
And then calling it as follows and re assigning the goal and start variables:
```python
skeleton = medial_axis(invert(grid))
skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)
grid_start = int(skel_start[0]), int(skel_start[1])
grid_goal = int(skel_goal[0]), int(skel_goal[1])
```
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I did a couople additiions/modifications to get the diagonals working.
a. Added new acitons to the class Action
```python
    SOUTHWEST = (1, -1, 1.41)
    NORTHWEST = (-1, -1, 1.41)
    SOUTHEAST = (1, 1, 1.41)
    NORTHEAST = (-1, 1, 1.41)
```
b. Added valid actions to account for diagonal movement in the valid actions function.
```python
 if grid[x - 1, y - 1] == 1:
         valid_actions.remove(Action.NORTHWEST)
    if grid[x + 1, y - 1] == 1:
         valid_actions.remove(Action.SOUTHWEST)
    if grid[x - 1, y + 1] == 1:
         valid_actions.remove(Action.NORTHEAST)
    if grid[x + 1, y + 1] == 1:
         valid_actions.remove(Action.SOUTHEAST) 
```
Here is a picture of my drone doing a diagonal
![Diagonal Path](./misc/HomeLocation.png)
#### 6. Cull waypoints 
For this step I used the collinearity check and path prunning fucntions that we learened in the lessons to reduce the number of waypoints.
```python
def collinearity_check(p1, p2, p3, epsilon=1e-6):
    #Collinearity_check from lessons   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    #Prune Path funciton from lessons
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

```
And here is where I called the functions:
```python
pruned_path = prune_path(path)
```
Here I used the pruned_path to caluclate the waypoints:

```python    
waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
```

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


