# dora-husky-rcs

A dora node implementing a GRPC remote control server for Husky A200 Robots. Since the Husky A200 uses ROS2, dora-husky-rcs uses the ROS2 bridge offered by dora-rs to publish and subscribe to the different ROS2 topics of the Husky bot.

The goal of this project was for us at CARVI to be able to teleoperate a husky A200 robot from our teleoperation application built in Flutter.

## Setup

For the moment the only supported setup is running Dora locally on the Husky A200. Since Zenoh is only supported by the A200 at the moment, you will need to set up ROS2 on your offboard pc and use Fast-RTPS discovery as the communication method.

1. Install dora-rs on your Husky A200 robot. Follow the instructions in the [dora-rs](https://dora-rs.ai/).

2. install UV:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

3. Clone this repository on your Husky A200 robot.

```bash
git clone https://github.com/groupe-carvi/dora-husky-rcs.git
```

4. Build the dora dataflow:

```bash
cd dora-husky-rcs
dora build ./dataflow.yml
```

## References
- [Husky Platform ROS API](https://docs.clearpathrobotics.com/docs/ros/api/platform_api)
