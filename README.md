# dora-husky-rcs

A dora node implementing a GRPC remote control server for Husky A300 Robots. Since the Husky A300 uses ROS2, dora-husky-rcs uses the ROS2 bridge offered by dora-rs to publish and subscribe to the different ROS2 topics of the Husky bot.

The goal of this project was for us at CARVI to be able to teleoperate a husky a300 robot from our teleoperation application built in Flutter.

## Setup

There is two possible setup we support in this implementation:

1) Running Dora localy on the Husky A300. The simpler one and should not need a lot of configuration to do.
2) Running on a separate host with zenoh as middleware. (Supported by ROS2 starting with Jazzy)


### Offboard setup using zenoh
If you want to communicate to the Husky from an offboard computer, you will need make sure dora is able to send remote communication and that can be done with [Zenoh](https://zenoh.io/). Zenoh act as communication middleware and ROS2 offer an RMW implementation of it starting on Jazzy and there is also some support already on the Husky.

```shell
# On the Husky you'll need to start the Zenoh router
# manually using this:
bash -e ~/clearpath/zenoh-router-start
# Or it can be daemonized via systemctl using this:
source /opt/ros/jazzy/setup.bash
sudo systemctl start clearpath-zenoh-router
```
