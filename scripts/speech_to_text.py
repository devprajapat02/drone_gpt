#!/usr/bin/env python3

import rospy
import sounddevice as sd
import rospkg
import os
import vosk
import base64
import json

from turtle_gpt.srv import speechToText, speechToTextResponse

class SpeechToText:
    def __init__(self):
        rospy.init_node('stt_node')
        self.service = rospy.Service("stt", speechToText, self.handler)

        ## vsok models (https://alphacephei.com/vosk/models)
        model_name = "vosk-model-small-en-us-0.15"
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('turtle_gpt')
        models_dir = os.path.join(package_path, 'models')
        model_path = os.path.join(models_dir, model_name)
        self.model = vosk.Model(model_path)

        input_dev_num = sd.query_hostapis()[0]['default_input_device']
        device_info = sd.query_devices(input_dev_num, 'input')
        self.samplerate = int(device_info['default_samplerate'])

    
    def speechRecognition(self, audio_data):
        rec = vosk.KaldiRecognizer(self.model, self.samplerate)
        result_text = ""
        if rec.AcceptWaveform(audio_data):
            result = rec.FinalResult()
            diction = json.loads(result)
            if len(diction["text"]) > 2:
                result_text = diction["text"]
            rec.Reset()
        return result_text

    def handler(self, data):
        audio = base64.b64decode(data.b64_audio)
        result_text = self.speechRecognition(audio)
        # res = speechToTextResponse(result_text)
        return result_text

if __name__ == '__main__':
    SpeechToText()
    rospy.spin()