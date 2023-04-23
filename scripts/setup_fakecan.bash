#enable the fake can mod in linux:

sudo modprobe vcan

sudo ip link add dev vcan0 type vcan

sudo ip link set up vcan0
