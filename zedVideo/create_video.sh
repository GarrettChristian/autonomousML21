#!/bin/bash
echo "Creating video from svo file"

fileName=$(date +"zed_recording_%T")

if [ -z "$1" ]
then
    echo "No file name provided defualting to $fileName"
else
    fileName=$1
    echo "with name: $fileName"
fi

fileName="${fileName}.avi"

# stop the recording
# rosservice call /zed/zed_node/stop_svo_recording

# Create the video 
python3 svo_export.py new_svo.svo bin/original_zed_video.avi 1

# flip vertically
# ffmpeg -i bin/original_zed_video.avi -vf "vflip" bin/flip_zed_video.avi

# crop
# "[0]crop=iw/2:ih:0:0"
ffmpeg -i bin/original_zed_video.avi -filter_complex "[0]crop=iw/2:ih:ow:0" bin/crop.avi 

# Combine wav and avi
ffmpeg -i bin/crop.avi -i audio_zed.wav -c:v copy -c:a aac $fileName

# clean up excess avi videos
rm -rf bin/*
