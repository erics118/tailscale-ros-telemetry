# tailscale-ros

tailscale automation scripts for usage in ROS

`justfile` is just for development purposes.

`launch.sh` is a fully self-contained script that is used to set up tailscale.
requires `curl`, `jq`, and `tailscale` to be installed on the system, but it
will install them if they are not found.
