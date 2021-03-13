# How To Implement A Custom Solver Algorithm

[doogie_algorithms] allows choosing at runtime wich solver (global planner) will be use by the application to solve the maze, not being necessary any internal change on code structure to support new solvers algorithms. This is achived by using the ROS [pluginlib], where by using inheritance principles one may create an solver extension by making an external classes that inherites from the base class plugin, `doogie_algorithms::BaseSolver`.

---

**Note:** If you're not very familiar with pluginlib structure, you migh wanna see this [pluginlib tutorial] first.

---

## 1. The Base Class Structure

`doogie_algorithms::BaseSolver` already comes with most of its methods already default implemented, but it still allows overriding. Essenetially, only two of them are really required to be implemented in a new solver class: `bool makePlan()` and `bool move()`.

The first one is the solver algorithm per si, where you will plan your next next goal movement, applying your custom heuristics based on the current constraints (such as the wall presence). The methods `bool isWall[Front|Back|Left|Right]()` will give you the constraints that will be updated on each iteration of the [maze_solver_node]'s control loop.

The `bool move()`, on the other hand is a simple method to send the next goal to Doogie's action server, [move_in_maze], wich will receives three informations as input: direction, cells and turns, you may have a look at [DoogieMove.action] for a better understanding of the goal structure. After your goal is properly set, by using the class member `goal_`, you can send it through the [doogie_core::DoogieHandle] class, by calling `getDoogieHandle().move(goal_)`.

In addition, the base class allows configuration by using the ROS's [Parameter Server]. Extra parameters can be loaded by overridind the `void doogie_algorithms::BaseSolver::loadParams()`. Also the `struct doogie_algorithms::BaseSolver::ROSParams` offers an easy approach to store all your parameters as class members and new parameters can be added by just extending it into a new struct using public inheritance.

## 2. Hands-On

For a better understanding, let's make a simple implementation for `doogie_algorithms::BaseSolver`, using the right hand solver. For this section, all code will be considered implemented inside `doogie_algorithms`, within a package called `right_hand_solver`. You may want have a look into ROS wiki's tutorial [CreatingPackage] if you are not familiar with catkin package structure.

---

**Note:** Solver's plugins can be implemented into any catkin package, and it's recomended to be implemented outside `doogie_algorithms` folder structure. All code described here can be found at [right_hand_solver_plugin].

---

### 2.1 Class Definition

First, let's create the `include/right_hand_solver_plugin/right_hand_solver_plugin.hpp`.

```c++
  1  #ifndef DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
  2  #define DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
  3  
  4  #include <ros/ros.h>
  5  #include "doogie_algorithms/base_solver.hpp"
  6  #include "doogie_msgs/MazeCellMultiArray.h"
  7  #include "doogie_msgs/DoogiePose.h"
  8  
  9  namespace doogie_algorithms {
 10  
 11  namespace right_hand_solver_plugin {
 12  
 13  
 14  class RightHandSolverPlugin : public doogie_algorithms::BaseSolver {
 15   public:
 16    RightHandSolverPlugin() = default;
 17    void initialize() override;
 18    bool makePlan() override;
 19    bool move() override;
 20    bool isPlanAttemptsReached(int count);
 21  
 22   private:
 23    void loadParams();
 24    struct ROSParams : doogie_algorithms::BaseSolver::ROSParams {
 25      int plan_attempts{5};
 26    } params_;
 27  };
 28  
 29  }  // namespace right_hand_solver_plugin
 30  }  // namespace doogie_algorithms
 31  
 32  #endif  // DOOGIE_ALGORITHMS_RIGHT_HAND_PLUGIN_HPP_
```

Let's have a close look into the most important partes, breaking them down piece by piece.

```cpp
 17    void initialize() override;
 18    bool makePlan() override;
 19    bool move() override;
```

As shown in [The Base Class Structure](##1-the-base-class-structure), plugins should always override `bool makePlan()` and `bool move()` methods. But it's also overriding the `void initialize()` since this solver will load an extra parameter that will take the maximum plan attempts it should try before aborting it's execution. This is a good strategy to avoid the robot being stucked in an endless loop.

```cpp
 20    bool isPlanAttemptsReached(int count);
```

This method will be responsible for tracking down how many attempts solver has already made and check if it is still bellow the limit set.

```cpp
 23    void loadParams();
```

Here is where all parameters will be properly configured.

```cpp
 24    struct ROSParams : doogie_algorithms::BaseSolver::ROSParams {
 25      int plan_attempts{5};
 26    } params_;
```

Here, `doogie_algorithms::BaseSolver::ROSParams` is being extended to also store our maximum plan attempts parameter (`plan_attempts`). If no parameter is passed, it will be loaded with a maximum of five attempts by default.

### 2.2 Class Implementation

Now, we shall create our .cpp file `src/right_hand_solver_plugin.cpp`.

```cpp
  1  #include <pluginlib/class_list_macros.h>
  2  
  3  #include "doogie_algorithms/right_hand_solver_plugin/right_hand_solver_plugin.hpp"
  4  
  5  namespace doogie_algorithms {
  6  
  7  // register RightHandSolver as a BaseSolver implementation
  8  PLUGINLIB_EXPORT_CLASS(right_hand_solver_plugin::RightHandSolverPlugin, doogie_algorithms::BaseSolver)
  9  
 10  namespace right_hand_solver_plugin {
 11  
 12  void RightHandSolverPlugin::initialize() {
 13    loadParams();
 14  }
 15  
 16  void RightHandSolverPlugin::loadParams() {
 17    BaseSolver::loadParams(params_);
 18    if(!getPrivateNodeHandle().getParam("plan_attempts", params_.plan_attempts)) {
 19      ROS_INFO_STREAM("/plan_attempts param is not set. Using default: " << params_.plan_attempts);
 20    }
 21  }
 22  
 23  bool RightHandSolverPlugin::makePlan() {
 24    int plan_attempts = 0;
 25    while (!isPlanAttemptsReached(plan_attempts)) {
 26      if (!isWallRight()) {
 27        goal_.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
 28        ROS_INFO_STREAM("Goal is: right");
 29  
 30        return true;
 31      }
 32  
 33      if (!isWallFront()) {
 34        goal_.direction = doogie_msgs::DoogieMoveGoal::FRONT;
 35        ROS_INFO_STREAM("Goal is: front");
 36        return true;
 37      }
 38  
 39      if (!isWallLeft()) {
 40        goal_.direction = doogie_msgs::DoogieMoveGoal::LEFT;
 41        ROS_INFO_STREAM("Goal is: left");
 42        return true;
 43      }
 44  
 45      if (!isWallBack()) {
 46        goal_.direction = doogie_msgs::DoogieMoveGoal::BACK;
 47        ROS_INFO_STREAM("Goal is: back");
 48        return true;
 49      }
 50  
 51      plan_attempts++;
 52      ros::spinOnce();
 53    }
 54  
 55      return false;
 56  }
 57  
 58  bool RightHandSolverPlugin::isPlanAttemptsReached(int count) {
 59    if (count >= params_.plan_attempts) {
 60      ROS_FATAL("Could not make a plan. There is no path robot can move");
 61      throw std::runtime_error("");
 62    }
 63    return count >= params_.plan_attempts;
 64  }
 65  
 66  bool RightHandSolverPlugin::move() {
 67    goal_.cells = 1;
 68    getDoogieHandle().move(goal_);
 69  
 70    return true;
 71  }
 72  
 73  }  // namespace right_hand_solver_plugin
 74  }  // namespace doogie_algorithms
```

Let's have a look on how this implementation works

```cpp
  8  PLUGINLIB_EXPORT_CLASS(right_hand_solver_plugin::RightHandSolverPlugin, doogie_algorithms::BaseSolver)
```

`PLUGINLIB_EXPORT_CLASS` allow you to register your new plugin class, as described in [pluginlib tutorial]. Without this, loading the plugin in run time would not be possible.

```cpp
 12  void RightHandSolverPlugin::initialize() {
 13    loadParams();
 14  }
```

The initialization step will be necessary for making any configuration setting (such as loading the plugin's parameters) before starting the solver.

```cpp
 16  void RightHandSolverPlugin::loadParams() {
 17    BaseSolver::loadParams(params_);
 18    if(!getPrivateNodeHandle().getParam("plan_attempts", params_.plan_attempts)) {
 19      ROS_INFO_STREAM("/plan_attempts param is not set. Using default: " << params_.plan_attempts);
 20    }
 21  }
```

The `void RightHandSolverPlugin::loadParams()` allows loading the base class parameters by calling the default implementation `BaseSolver::loadParams()` and then configures our custom one into `params_.plan_attempts`.

```cpp
 23  bool RightHandSolverPlugin::makePlan() {
 24    int plan_attempts = 0;
 25    while (!isPlanAttemptsReached(plan_attempts)) {
 26      if (!isWallRight()) {
 27        goal_.direction = doogie_msgs::DoogieMoveGoal::RIGHT;
 28        ROS_INFO_STREAM("Goal is: right");
 29  
 30        return true;
 31      }
 32  
 33      if (!isWallFront()) {
 34        goal_.direction = doogie_msgs::DoogieMoveGoal::FRONT;
 35        ROS_INFO_STREAM("Goal is: front");
 36        return true;
 37      }
 39      if (!isWallLeft()) {...}
 44  
 45      if (!isWallBack()) {...}
 50  
 51      plan_attempts++;
 52      ros::spinOnce();
 53    }
 54  
 55      return false;
 56  }
```

Each time Doogie has moved, a new plan should be made for the next loop iteraction, if the robot has not achieve the maze's end. The plan construction and storage is done through the `goal_` object.

The plan's configuration is made inside this method so it can check where the current cell is not walled and then chose the direction the robot will move.

```cpp
 66  bool RightHandSolverPlugin::move() {
 67    goal_.cells = 1;
 68    getDoogieHandle().move(goal_);
 69  
 70    return true;
 71  }
```

Finally, just send the goal trough the move in maze server, by using the doogie handler.

### 2.3 Export The Library

Now your plugin is made, it should be properly found by the maze solver's node. It is a simple task to do, first make sure you have this on your CMakeLists.txt:

```Makefile
catkin_package(
  INCLUDE_DIRS 
    include/right_hand_solver_plugin
  LIBRARIES 
    right_hand_solver_plugin
  )

add_library(right_hand_solver_plugin
  src/right_hand_solver_plugin.cpp
)
```

Add a plugin description, using the library name you choose prefixed by lib (ie., `libright_hand_solver_plugin`).

```xml
<!-- right_hand_solver_plugin_description.xml -->
<library path="lib/libright_hand_solver_plugin">

  <class name="doogie_algorithms/RightHandSolverPlugin" type="right_hand_solver_plugin::RightHandSolverPlugin" base_class_type="doogie_algorithms::BaseSolver">
    <description>
      A right-hand solver for micromouse maze.
    </description>
  </class>
  
</library>
```

And finally export it in your package manifest by adding the export tag:

```xml
  <export>
    <doogie_algorithms plugin="${prefix}/right_hand_solver_plugin_description.xml"/>
  </export>
```

### 3. Use your new solver
[maze_solver_node] is the instance responsible for planning and send goals to Doogie so it can find it way out of the maze. In this node is where the plugin will be loaded.

As you can see there are two global strings, `BASE_CLASS_PACKAGE` and `BASE_CLASS`. So just override it with your plugin configuration, in the case of our example using the righ hand solver it would be:

```cpp
const std::string& BASE_CLASS_PACKAGE = "right_hand_solver_plugin";
const std::string& BASE_CLASS = BASE_CLASS_PACKAGE+"::RightHandSolverPlugin";
```

After this change, rebuild doogie_navigation package then just launch [sim_solver_application] to use the solver in the simulation environment.

[CreatingPackage]: http://wiki.ros.org/ROS/Tutorials/CreatingPackage
[doogie_algorithms]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/tree/master/doogie_algorithms
[doogie_algorithms]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/tree/master/doogie_core
[DoogieMove.action]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/move_base/no-pid/doogie_msgs/action/DoogieMove.action
[maze_solver_node]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/master/doogie_navigation/src/maze_solver_node.cpp
[move_in_maze]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/blob/move_base/no-pid/doogie_navigation/include/doogie_navigation/move_in_maze.hpp
[Parameter Server]: http://wiki.ros.org/Parameter%20Server
[pluginlib]: http://wiki.ros.org/pluginlib
[pluginlib tutorial]: http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
[right_hand_solver_plugin]: https://github.com/Brazilian-Institute-of-Robotics/doogie_base/tree/master/doogie_algorithms
[sim_solver_application]: https://github.com/Brazilian-Institute-of-Robotics/doogie_simulator/blob/master/doogie_gazebo/launch/sim_solver_application.launch
