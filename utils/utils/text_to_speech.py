from gtts import gTTS
from pygame import mixer  # Load the popular external library
import time

def text_to_speech(stuff):
    tts=gTTS(text=stuff,lang="en")
    tts.save("hello.mp3")

    mixer.init()
    mixer.music.load('hello.mp3')
    mixer.music.play()
    while mixer.music.get_busy():
        time.sleep(1)

text_to_speech("hello world")

