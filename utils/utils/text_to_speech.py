from gtts import gTTS
# from pygame import mixer
# import time
import json 

import playsound

speech_to_title = {}

def text_to_speech(speech, title):
    if speech not in speech_to_title:
        speech_to_title[speech] = title

        json_object = json.dumps(speech_to_title, indent=4)
        with open("utils/utils/speech_to_title.json", "w") as outfile:
            outfile.write(json_object)
    
    tts=gTTS(text=speech,lang="en")
    t = ""
    with open('utils/utils/speech_to_title.json', 'r') as openfile:
        json_object = json.load(openfile)
        t = json_object[speech]
    final_title = f"utils/audio/{t}.mp3"
    tts.save(final_title)

    # mixer.init()
    # mixer.music.load(final_title)
    # mixer.music.play()
    # while mixer.music.get_busy():
    #     time.sleep(5)
    playsound.playsound(final_title, True)

text_to_speech("you did not bet enough, please place down more chips","rebet")