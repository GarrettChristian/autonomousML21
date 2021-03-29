#!/bin/bash
echo "Creating video from svo file"

# stop the recording
rosservice call /zed/zed_node/stop_svo_recording

# Create the video 
python3 svo_export.py new_svo.svo bin/original_zed_video.avi 1

# flip vertically
ffmpeg -i bin/original_zed_video.avi -vf "vflip" bin/flip_zed_video.avi

# crop
ffmpeg -i bin/flip_zed_video.avi -filter_complex "[0]crop=iw/2:ih:0:0" final.avi 

# clean up excess avi videos
rm -rf bin/*