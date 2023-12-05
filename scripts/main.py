#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
import sounddevice as sd
import base64

from std_msgs.msg import String
from turtle_gpt.srv import GPTPrompt, speechToText

class SpeechToTextGUI:
    def __init__(self, master):
        self.master = master
        master.title("Speech To Text")

        self.label = ttk.Label(master, text="Press Start/End to begin/complete recording...")
        self.label.pack(pady=10)

        # Start Button
        self.start_button = ttk.Button(master, text="Start", command=self.onStart)
        self.start_button.pack(pady=10)

        # End Button
        self.end_button = ttk.Button(master, text="End", command=self.onEnd)
        self.end_button.pack(pady=10)

        # Speech to Text service
        self.stt_service = rospy.ServiceProxy('/stt', speechToText)

        # Default Parameters
        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        self.device_info = sd.query_devices(self.input_dev_num, 'input')
        self.samplerate = int(self.device_info['default_samplerate'])

        # Store the raw audio bytes
        self.raw_audio_data = bytearray()

        # raw audio stream
        self.stream = sd.RawInputStream(samplerate=self.samplerate, channels=1, dtype='int16', blocksize=16000, callback=self.onAudio)        
        self.recording_id = 0

    def onAudio(self, indata, frames, time, status):
        self.raw_audio_data.extend(indata)

    def onStart(self):
        self.resetAudioData()
        self.recording_id += 1
        self.stream.start()

    def onEnd(self):
        self.stream.stop()
        if self.recording_id == 1:
            # if recording ended first time, display the convert button
            self.rec_comp_label = ttk.Label(self.master, text=f"Recorded Audio({len(self.raw_audio_data)} bytes)")
            self.rec_comp_label.pack(pady=10)

            self.convert_button = ttk.Button(self.master, text="Convert", command=self.onConvert)
            self.convert_button.pack(pady=10)
        else:
            # update label on successive recordings
            self.rec_comp_label.config(text=f"Recorded Audio({len(self.raw_audio_data)} bytes)")
    
    def onConvert(self):
        # convert bytearray to string
        audio_encode = base64.b64encode(self.raw_audio_data).decode('utf-8')
        res = self.stt_service(audio_encode)
        rospy.loginfo(res.text)
        text = res.text
        if text!="":
            publisher = rospy.Publisher("turtle_command", String, queue_size=1)
            rospy.wait_for_service('/GPTPrompt')
            GPTService = rospy.ServiceProxy('/GPTPrompt', GPTPrompt)
            res = GPTService(text)
            result = String()
            result.data = res.response
            publisher.publish(result)

        
    def resetAudioData(self):
        try:
            self.raw_audio_data = bytearray()
        except Exception as e:
            print(e)

def handleCMD():
    publisher = rospy.Publisher("turtle_command", String, queue_size=1)
    while not rospy.is_shutdown():
        inp = input("Enter the command for turtlesim: ")
        rospy.wait_for_service('/GPTPrompt')
        GPTService = rospy.ServiceProxy('/GPTPrompt', GPTPrompt)
        res = GPTService(inp)

        result = String()
        result.data = res.response
        # result.data = '''[
        #     {"action": "rotateByTheta", "params": { "theta": 2, "direction": "clockwise" }},
        #     {"action": "moveByDistance", "params": { "distance" : 1 }},
        #     {"action": "goToGoal", "params": { "goal_x" : 1, "goal_y": 1, "goal_theta": 1.57  }}
        # ]'''
        publisher.publish(result)

def handleGUI():
    root = tk.Tk()
    gui = SpeechToTextGUI(root)
    root.mainloop()

def main():     
    type = input("Press (1) - for Speech to Text GUI, (2) - for commandline")
    if type == '1':
        handleCMD()
    elif type == '2':
        handleGUI()



if __name__ == "__main__":
    rospy.init_node('PromptNode',anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass