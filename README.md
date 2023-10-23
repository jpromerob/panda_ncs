# Operate Hammer Follower:

## Start Optitrack:

```
cd /opt/Panda/Code/panda_ncs
remmina optitrack.rdp 
```
Open Exceptional.cal with Motive

### Check Hammer Tracking:

```
cd ~/GenGTD
python3 live_pose.py <N>
```

Where N (N in {1,2,3}) is the ID of the camera to be inspected.


## Operate Panda:

### Enable Robot
Go to Panda Desk: https://172.16.0.2/desk/
Unlock joints
Set 'activated' mode (using black button on desk)

### Run program

```
cd /opt/libfranka/build
sudo cmake --build .
./panda_ncs/xyz_follower "172.16.0.2"
```
