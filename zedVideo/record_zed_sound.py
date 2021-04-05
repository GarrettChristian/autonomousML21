# https://github.com/stereolabs/zed-ros-wrapper/issues/371
# https://github.com/stereolabs/zed-ros-wrapper/issues/348

import rospy
import sys
import os
import subprocess
import pyaudio
import wave
from zed_interfaces.srv import *


def main():

    # Set up audio
    audio = pyaudio.PyAudio()
    fname = "audio_zed.wav"
    frmat = pyaudio.paInt16
    channels = 1
    rate = 44100
    chunk = 1024
    
    frames = []
    stream = audio.open(format = frmat, 
                    channels = channels, 
                    rate = rate, 
                    input = True, 
                    frames_per_buffer = chunk)

    
    # Record video
    rospy.wait_for_service('/zed/zed_node/start_svo_recording')

    try:
        start_recording = rospy.ServiceProxy('/zed/zed_node/start_svo_recording', start_svo_recording)
        resp1 = start_recording('/home/jacart/catkin_ws/src/autonomousMLgc/zedVideo/new_svo.svo')
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


    try:
        # Record Audio
        print("Recording Audio")
        while True:
            data = stream.read(chunk)
            frames.append(data)
        
    except KeyboardInterrupt:
        print("Saving audio and svo")

        # Save video
        rospy.wait_for_service('/zed/zed_node/stop_svo_recording')

        try:
            stop_recording = rospy.ServiceProxy('/zed/zed_node/stop_svo_recording', stop_svo_recording)
            resp2 = stop_recording()
            print(resp2)
        except rospy.ServiceException as e:
            print("Service call failsed: %s"%e)

        # save audio 
        stream.close()
        wf = wave.open(fname, 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(audio.get_sample_size(frmat))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))
        wf.close()


if __name__ == "__main__":
    main()
