# Arduino UNO Taiko no Tatsukjin Controller Tutorial

- **Aluna:** Julia Karine Peres
- **Curso:** Ciencia da Computação
- **Semestre:** 8
- **Contato:** juliakp1@al.insper.edu.br
- **Ano:** 2025

## Software Setup - Arduino IDE

Install the latest version of Arduino IDE from the official website: https://www.arduino.cc/en/Main/Software

## Circuit Setup

### Materials

To set up the circuit, you need an Arduino UNO, a spongeboard, a set of four buttons/switches, and 13 male-male connection wires.

### Connect the Circuit
Connect the wires and sensors according to the diagram below:


## Code

In the Arduino RDE, select tools -> board -> UNO, then select tools -> ports and the port the UNO is currently connected to.

Paste the code below into the code editor and then select the right-pointing arrow that says upload:

<pre>
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
</pre>

After it finishes flashing the code, it should open a terminal where you can test them by seeing the prints 1 through 4.

Next, create a python file and paste the following code inside it:

<pre>
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
</pre>

You will probably need to alter the PORT variable to the port you selected when correcting the UNO to your PC.

Install the dependencies:

<pre>
pip install pyserial pynput
</pre>

And then you can run the script! It should already be working if the keybinds match the game you're playing. If not, either change the keybinds in the script or in the game.

## Upgrades 

Want to use an actual drumming motion with this controller? Follow the section below!

### Setup

The connection from the board to sponge board is the same, you just need to switch the buttons to piezo sensors or similar vibration detectors. 

For best performance, the sensors must be piezo sensors (a.k.a. piezo speakers, contact microphones). No guarantee if other types of sensors will simply work, but if analog signals with voltage ranged 0-5V are fed into analog pins, this setup should be good to go.

### Code

The python code will remain the same, but the arduino code will need to be switched to:

<pre>
const int min_threshold = sq(200); // The threshold is betwee 0-1024

const int pin[4] = {A0, A1, A2, A3};
const int key[4] = {1,2,3,4};
int raw[4] = {0, 0, 0, 0};

bool Trig = false;
int key_index = 0;


void setup() {

#ifdef HID
  Keyboard.begin();
#endif
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
</pre>

## Credits

This project uses as a reference:
- Arduino Leonardo Taiko repository by LuiCat: https://github.com/LuiCat/ArduinoTaikoController
- Taiko Rythm Game Open Controller by Eymeric: https://www.instructables.com/Taiko-Rythm-Game-Open-Controller/