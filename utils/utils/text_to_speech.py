from gtts import gTTS
# from pygame import mixer
# import time
import json 

import playsound

from ament_index_python.packages import get_package_share_directory as pkgdir
import os

def text_to_speech(speech, title):

    final_title = f'/home/robot/robotws/src/PokerBot/utils/audio/{title}.mp3'
    glados_is = f'/home/robot/robotws/src/PokerBot/utils/audio/glados_is.mp3'
    if not os.path.exists(final_title):
        tts=gTTS(text=speech,lang="en")
        tts.save(final_title)
        
    #playsound.playsound(glados_is, True)
    playsound.playsound(final_title, True)

# # text_to_speech("you did not bet enough, please place down more chips","rebet")