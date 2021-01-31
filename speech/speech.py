import speech_recognition as sr

# following this guide 
# https://realpython.com/python-speech-recognition/#how-speech-recognition-works-an-overview
def main(): 
    r = sr.Recognizer()  
    with sr.Microphone() as source:
        print("Please wait. Calibrating microphone...")  
        # listen for 5 seconds and create the ambient noise energy level  
        r.adjust_for_ambient_noise(source, duration=5)  
        print("Say something!")  
        audio = r.listen(source)

    # recognize speech using Sphinx  
    try:  
        print("Sphinx thinks you said '" + r.recognize_sphinx(audio) + "'")  
    except sr.UnknownValueError:  
        print("Sphinx could not understand audio")  
    except sr.RequestError as e:  
        print("Sphinx error; {0}".format(e)) 
  
if __name__=="__main__": 
    main() 