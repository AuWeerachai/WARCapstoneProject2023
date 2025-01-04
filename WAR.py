import sys
from window import Window

DIMENSION = (1366, 768)
MIN_SIZE = (1000, 665)

class App:
    def __init__(self):
        if len(sys.argv) > 1:
            if ("--debug" in sys.argv) or ("-d" in sys.argv):
                # just run GUI without calling other functions
                print("[war] Debugging mode active")
                self.window = Window(DIMENSION, MIN_SIZE, debug=True)
            else:
                print(f"[war] Invalid Argument: {sys.argv[1]}. Ignoring...")
                self.window = Window(DIMENSION, MIN_SIZE)
        else:
            self.window = Window(DIMENSION, MIN_SIZE)

if __name__ == '__main__':        # if(self.group.family_name == "Arm1"):
        #     print("THIS IS ARM1")
        # elif(self.group.family_name == "Arm2"):
        #     print("THIS IS ARM2")
    app = App()