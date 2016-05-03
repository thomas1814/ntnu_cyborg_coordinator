# ntnu_cyborg_coordinator

The `ntnu_cyborg_coordinator` is a ROS catkin package for the NTNU Cyborg project. It coordinates `RosAria` control between multiple users.

The following packages must be available to run `ntnu_cyborg_coordinator`:

* https://github.com/thentnucyborg/rosaria

## Launching coordinator

`roslaunch ntnu_cyborg_coordinator coordinator`

This will launch the `RosAria` node and the `ntnu_cyborg_coordinator` node.

## Requesting control

A user can request robot control by calling the `RequestControl` service with an `id` argument describing the user:

```
rosservice call /ntnu_cyborg_coordinator/requestControl my_user_identifier
controlReceived: True
```

The service call will return a boolean value `controlReceived` indicating whether the user has been given control of the the robot. A user will remain in control until it releases control through the `ReleaseControl` service.

## Controlling the robot

Normally, the user would send robot control messages to `/RosAria/cmd_vel`. When using the `ntnu_cyborg_coordinator`, messages are instead sent to `/ntnu_cyborg_coordinator/my_user_identifier/RosAria/cmd_vel`. If the user has been given control over the robot as the result of a `requestControl` call, the `ntnu_cyborg_coordinator` will route messages from the user specific topic to `/RosAria/cmd_vel`.

## Releasing control

A user in control of the robot can release their control by calling the `ReleaseControl` service with the same `id` argument used when calling `RequestControl`:

```
rosservice call /ntnu_cyborg_coordinator/releaseControl my_user_identifier
```

## References

* http://www.ros.org/
* https://www.ntnu.edu/cyborg/student-organization/
