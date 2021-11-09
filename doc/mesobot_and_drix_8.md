# Mesobot and Drix 8 integration using Project11

Details on Project11 on Drix 8 can be found in [drix_8.md](drix_8.md).

Drix 8's backseat driver, echo, is configured to launch Project11 on boot including the Sonardyne USBL nodes and the Mesobot node.

It expects Sonardyne's Ranger software to be running and properly configured on survey_pc.

## Ranger software

Verify that beacons are activly tracked and power settings are appropriate for the conditions.

## Operator station

Mesobot does not show up automatically in camp. Using the `drix_project11` package's `operator_ui.launch` file, the `mesobot` argument must be spcifed to be true.

    mon launch drix_project11 operator_ui.launch mesobot:=true

## Troubleshooting

On echo, verify that the nodes are running properly.

    tmux attach -t project11

Verify that the USBL nodes are publishing data.

    rostoepic echo /project11/drix_8/usbl/geographic_positions

Verify that positions with heading are published from the Mesobot node.

    TODO: rostopic echo

Verify that Mesobot topics are being sent by the UDP bridge.

    rosrun udp_bridge udp_bridge_ui

Verify that the same topics show up on the operator station.

## References

- [drix_8.md](drix_8.md)
- [Mesobot Project11 package](../../../../mesobot_project11)
- [Sonardyne USBL ROS package](../../../../sonardyne_usbl)