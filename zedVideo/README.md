# Making recordings using the zed wrapper 

- going to call the zed wrapper make video at the same time as I tell the audio to record then use ffmeg to crop flip and add audio to the avi

- installed: 
- sudo apt install ffmpeg
- zed python wrapper in /usr/local/zed/get_python_api.py *had to change os version to ubuntu18

- launch zed
- roslaunch zed_wrapper zed.launch

- start
- rosservice call /zed/zed_node/start_svo_recording test.svo
 or
 rosservice call /zed/zed_node/start_svo_recording /home/jacart/catkin_ws/src/autonomousMLgc/zedVideo/new_svo.svo
- stop
- rosservice call /zed/zed_node/stop_svo_recording

- export
- python3 svo_export.py test.svo out_svo.avi 1

vertical flip
- https://askubuntu.com/questions/83711/how-can-i-rotate-a-video
- https://duxyng.wordpress.com/2013/04/07/rotateflip-video-with-ffmpeg/
- ffmpeg -i out_svo.avi -vf "vflip" out.avi

crop 
- https://linuxhint.com/crop_videos_linux/
- https://stackoverflow.com/questions/52582215/commands-to-cut-videos-in-half-horizontally-or-vertically-and-rejoin-them-later
- ffmpeg -i out.avi -filter_complex "[0]crop=iw/2:ih:0:0" left.avi 

overwite
- https://stackoverflow.com/questions/39788972/ffmpeg-override-output-file-if-exists
