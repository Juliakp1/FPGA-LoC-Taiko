# Arduino UNO Taiko no Tatsukjin Controller Tutorial

!!! info
      - Julia Karine Peres
      - Ciencia da Computação
      - juliakp1@al.insper.edu.br
      - 2025

## Why this project?

  I am a big rhythm game fan, and mostly play on my PC. About 2 years ago, I went to an arcade that had a taiko machine: the [Wadiko Master](https://taiko.fandom.com/wiki/Wadaiko_Master), which is actually a Brazilian exclusive verison! I really liked how you were able to actually drum out the notes insted of just pressing keyboard keys, even if it took a bit of getting used to.

  I went to search online to see if there was a 'controller' i could buy that would allow me to drum out the notes at home, allowing me to play any song with it and not need to play a per-play fee. I found 2 types: the small, cheap feeling drums that are mostly used for the console releases of the game, [like this one](https://www.amazon.com.br/OSTENT-Tatsujin-compat%C3%ADveis-Nintendo-videogame/dp/B00FJ2JX2U), and the high-end 'arcade-like experience' drums that cost \$500+ (R\$4.000+), like the [Yuakon Taiko Force](https://yuancon.store/controller/taiko).

  Both of them felt way too expensive for the experience they delivered, and i wanted to try making one for myself since. This project is an attempt to make such a controller, but with the flexibility of allowing you to choose the size and material of your drum, and the flexibility to use buttons if you dont want/have piezo sensors. This drum is made for PC, and will not work of the physical console releases (though they probably work on PC emulators of said consoles).

  For refence, [this is what the arcade drums look like in action](https://www.youtube.com/watch?v=TpU-Nzul7Wk), though its not the same version as the brazilian one (center drum hits are red notes, rim hits are the blue notes).

## Software Setup - Arduino IDE

Install the latest version of Arduino IDE from the official website: https://www.arduino.cc/en/Main/Software

## Circuit Setup

### Materials

To set up the circuit, you need an Arduino UNO, a set of four buttons/switches, a spongeboard, and 9 male-male connection wires.

For more flexibility (expecially if you'll use piezo sensors, which we will see later), you can trade the spongeboard and the male-male wires for normal wires and soldering.

### Connect the Circuit
Connect the wires and sensors according to the diagram below:

![diagram](<diagram.png>)

## Code

In the Arduino RDE, select tools -> board -> UNO, then select tools -> ports and the port the UNO is currently connected to.

Paste the code below into the code editor and then select the right-pointing arrow that says upload:

``` c
const int button1 = A0;
const int button2 = A1;
const int button3 = A2;
const int button4 = A3;

int lastState1 = HIGH;
int lastState2 = HIGH;
int lastState3 = HIGH;
int lastState4 = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
}

void loop() {
  int state1 = digitalRead(button1);
  int state2 = digitalRead(button2);
  int state3 = digitalRead(button3);
  int state4 = digitalRead(button4);

  // Detect press (HIGH → LOW transition)
  if (state1 == LOW && lastState1 == HIGH) {
    Serial.println("1");
  }
  if (state2 == LOW && lastState2 == HIGH) {
    Serial.println("2");
  }
  if (state3 == LOW && lastState3 == HIGH) {
    Serial.println("3");
  }
  if (state4 == LOW && lastState4 == HIGH) {
    Serial.println("4");
  }

  // Update last states
  lastState1 = state1;
  lastState2 = state2;
  lastState3 = state3;
  lastState4 = state4;

  delay(50); // debounce delay
}
```

After it finishes flashing the code, it should open a terminal where you can test them by seeing the prints 1 through 4.

Next, create a python file and paste the following code inside it:

``` py
import serial
import time
from pynput.keyboard import Controller

# Change to your Arduino's COM port
PORT = "COM6"
BAUD_RATE = 9600

# Map button numbers to keys
KEY_MAP = {
    '1': 'k',
    '2': 'j',
    '3': 'f',
    '4': 'd',
}

ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
keyboard = Controller()

time.sleep(2)  # wait for Arduino to reset

print("Listening for button presses...")

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if line in KEY_MAP:
            key = KEY_MAP[line]
            print(f"Button {line} → Key '{key}'")
            keyboard.press(key)
            keyboard.release(key)  # quick tap
    except KeyboardInterrupt:
        print("Exiting...")
        break
    except Exception as e:
        print("Error:", e)
        continue
``` 

You will probably need to alter the PORT variable to the port you selected when correcting the UNO to your PC.

Install the dependencies:

``` sh
pip install pyserial pynput
```

And then you can run the script! It should already be working if the keybinds match the game you're playing. If not, either change the keybinds in the script or in the game.

!!! info
      The section below is for the piezo sensors, and will require you to connect the cables/components without a bread board!

## Upgrades 

Want to use an actual drumming motion with this controller? Follow the section below!

### Setup

The connection from the board to sponge board is the same, you just need to switch the buttons to piezo sensors or similar vibration detectors.

For best performance, the sensors must be piezo sensors (a.k.a. piezo speakers, contact microphones). No guarantee if other types of sensors will simply work, but if analog signals with voltage ranged 0-5V are fed into analog pins, this setup should be good to go.

If the hits are triggering at random and/or too much, putting a resistor that connects the Ax (A0 through A3) cable and the ground cable for each input should fix that issue.

### Code

The python code will remain the same, but the arduino code will need to be switched to:

``` c
const int min_threshold = sq(200); // The threshold is betwee 0-1024

const int pin[4] = {A0, A1, A2, A3};
const int key[4] = {1,2,3,4};
int raw[4] = {0, 0, 0, 0};

bool Trig = false;
int key_index = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {

  raw[0] = sq(analogRead(pin[0]));
  delayMicroseconds(200);
  raw[1] = sq(analogRead(pin[1]));
  delayMicroseconds(200);
  raw[2] = sq(analogRead(pin[2]));
  delayMicroseconds(200);
  raw[3] = sq(analogRead(pin[3]));
  delayMicroseconds(200);

  int energy =  raw[0] + raw[1] + raw[2] + raw[3];

  if( (energy > min_threshold)&&(!Trig) )
  {
    Trig = true;
    key_index = 0;
    int distance = energy-raw[0];
    for(int i = 0; i <= 3; i++)
    {
      int temp_distance = energy - raw[i];
      if( temp_distance < distance )
      {
        distance = temp_distance;
        key_index = i;
      }

    }
    Serial.println(key[key_index]);

  }

  if((energy< min_threshold)&&Trig)
  {
    Trig = false;
  }
}
``` 

## Credits

This project uses as a reference:

:heart: - Arduino Leonardo Taiko repository by LuiCat: https://github.com/LuiCat/ArduinoTaikoController

:heart: - Taiko Rythm Game Open Controller by Eymeric: https://www.instructables.com/Taiko-Rythm-Game-Open-Controller/

:heart: - Official Wadaiko Master Manual: www.adibra.com.br/Content/upload/downloads/634/manual-wadaiko.pdf 