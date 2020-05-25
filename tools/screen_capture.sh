#!/bin/sh
# author: Yannik Motzet (TIT17)
# email: yannik.motzet@outlook.com

# simple script which captures desktop screen and sends it to /dev/video0
# manual: https://superuser.com/questions/411897/using-desktop-as-fake-webcam-on-linux
# requirements:
# 	v4l2loopback - https://github.com/umlaeute/v4l2loopback
# 	ffmpeg
sudo modprobe v4l2loopback
# 720p@15fps
#ffmpeg -f x11grab -r 60 -s 1280x720 -i :0.0+0,0 -vcodec rawvideo -pix_fmt yuv420p -threads 0 -f v4l2 /dev/video0

# 1080p@60fps
#ffmpeg -f x11grab -r 60 -s 1920x1080 -i :0.0+0,0 -vcodec rawvideo -pix_fmt yuv420p -threads 0 -f v4l2 /dev/video

# 1080p@60fps
ffmpeg -f x11grab -r 15 -s 1920x1080 -i :0.0+0,0 -vcodec rawvideo -pix_fmt yuv420p -threads 0 -f v4l2 /dev/video0
