To mount the ssd to the data folder:

```
sudo mount -t ntfs /dev/sda1 ~/catkin_ws2/src/realsense_streamer/data/
```

This assumes the drive is located at /dev/sda1 - check with:

```
(sudo) fdisk -l
```