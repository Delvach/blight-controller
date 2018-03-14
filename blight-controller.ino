/*
   Master Effects Controller

   - This is a prototype device with:
       - Arduino Uno
       - Sparkfun Spectrum Analyzer
       - Wii Nunchuk controller
       - nRF24l01 wireless
       - 16-light RGB NeoPixel ring
   - Maintains data on lighting effects and broadcasts to child receivers
   - Receives commands from child receivers so controls aren't physically restricted to main transmitter
   - Will be transferring from an Uno to a Mega for next version
   - Next version will eliminate wii controller with more refined physical controls

*/

#include <BikeControlData.h>  // Data struct for receiving commands send to master module
#include <BikeEffectsData.h>  // Data struct for managing and sending effects data
#include <BikeEffectsUtils.h> // Common color and behaviors utilities 
#include <ArduinoNunchuk.h>     // Include wii nunchuk library
#include <Adafruit_NeoPixel.h>  // Include Neopixel library

#include "RF24.h" // Include nRF24L01 wireless chip library

#define behavior_debug_output false

// Sparkfun spectrum analyzer shield pins
#define SPECTRUMSHIELD_PIN_STROBE 4
#define SPECTRUMSHIELD_PIN_RESET 5
#define SPECTRUMSHIELD_PIN_LEFT A0 //analog
#define SPECTRUMSHIELD_PIN_RIGHT A1 //analog

int left[7]; // Left channel data 7 x 0-1023
int right[7]; // Right channel data 7 x 0-1023

// Define pin and count for ring (or strip) of Neopixel RGB lights
#define NEOPIXEL_PIN 9
#define NEOPIXEL_COUNT 16

/*
   Potentiometers
*/
#define POTENTIOMETER_PIN_HUE 2
#define POTENTIOMETER_PIN_AUDIO_MIN 3

byte num_pots = 2;
int pot_state[2] = {0, 0};
int pot_state_last[2] = {0, 0};
unsigned long pot_last_debounce_time[2] = {0, 0};


/*
   Buttons pins and debounce settings
*/
byte num_single_buttons = 4;
//int single_button_pins[4] = {2, 3, 6, 10};
int single_button_pins[4] = {6, 10, 3, 2};
byte single_button_state[4] = {0, 0, 0, 0};
byte single_button_curr_state[4] = {0, 0, 0, 0};
byte single_button_last_state[4] = {0, 0, 0, 0};
unsigned long single_button_last_debounce_time[4] = {0, 0, 0, 0};


/*
   Hue value is 0-359 but going from red to red feels redundant; clipping top end to limit to lavender
*/
short int hue_index_min = 0;
short int hue_index_max = 349;

short int audio_trim_min = 0;
short int audio_trim_max = 80;

short int brightness_setting_min = 10; // Sparkle has different needs than other modes; investigate
short int brightness_setting_max = 255;

// Read the 'minimum audio' potentiometer
int main_pot_audio_min = 0;


// What is main potentiometer currently controlling?
byte main_pot_setting = 0;

/*
   Data structures setup
*/

// Instantiate data structure for sending effects data
BikeEffectsData lights;

// Instantiate data structure for receiving commands from control units
BikeControlData receivedControlData;

/*
   Neopixels setup
*/
#define WRGB_NEO_PTYPE  NEO_GRBW
#define RGB_NEO_PTYPE  NEO_GRB
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, RGB_NEO_PTYPE + NEO_KHZ800);

/*
   NRF24L01 radio setup
*/
RF24 radio(7, 8);

/*
   Nunchuk controller setup
*/
ArduinoNunchuk nunchuk = ArduinoNunchuk();

/*
   Neopixel colors presets
*/
uint32_t BLACK = ring.Color(0, 0, 0);
uint32_t RED = ring.Color(31, 0, 0);
uint32_t GREEN = ring.Color(0, 31, 0);
uint32_t BLUE = ring.Color(0, 0, 31);
uint32_t PURPLE = ring.Color(15, 0, 13);

/*
   Wii nunchuk joystick interpreted readings
*/
int joystickOffset = 80;
byte joystickPositionIndex = 0;
byte pitch = 127;
byte roll = 127;


/*
   Debounce wii nunchuk buttons
*/
long debounce_delay = 200;

byte num_buttons = 2;
byte button_state[2] = {0, 0};
byte button_curr_state[2] = {0, 0};
byte button_last_state[2] = {0, 0};
unsigned long button_last_debounce_time[2] = {0, 0};

// Support long button presses
byte persistent_trigger_active = 0;
byte persistent_trigger_index = 0;
unsigned long persistent_trigger_last_updated = 0;
unsigned long persistent_trigger_update_interval = 200;


/*
   Behaviors
*/
bool behavior_auto_cycle_hue = true;
unsigned long update_interval = 20; // How often to update everything in milliseconds
unsigned long update_timestamp = 0; // Last time everything was updated in milliseconds timestamp

// Short timer uses micros() instead of millis() for faster behavior
unsigned long update_short_interval = 500;
unsigned long update_short_timestamp = 0;


/*
   Cycling position setting
   uses micros() instead of millis() for faster behavior
*/
unsigned long pos_last_update = 0;
unsigned long pos_change_interval_min = 500;
unsigned long pos_change_interval_max = 3000;
unsigned long pos_current_interval = 0;
byte pos_direction = 0;

unsigned long position_cycle_time = 1000;
unsigned long position_cycle_time_min = 100;
unsigned long position_cycle_time_max = 1000;
unsigned long position_cycle_time_started = 0;


/*
   Auto-cycling hue
*/
unsigned long auto_cycle_hue_last_update = 0;
unsigned long auto_cycle_hue_change_interval = 20;


/*
   Effect behavior - strobe
   NOTE: Limit the potential effect of flashing on epileptics:
   "Generally, flashing lights most likely to trigger seizures are
   between the frequency of 5 to 30 flashes per second (Hertz)."
*/
unsigned long strobe_last_update = 0;
unsigned long strobe_light_duration = 30;
unsigned long strobe_pause_duration = 300;


/*
   Neopixel timed display, shows color when hue changes and fades away
*/
unsigned long hue_change_last_update = 0;
unsigned long hue_change_display_time = 1000;
byte lightsFade_active = 0;
byte lightsFade_direction = 0;
byte lightsFade_lightIndex = 0;
byte lightsFade_fadeAll = 1;
int lightsFade_hue = 0;
byte lightsFade_val = 0;
unsigned long lightsFade_start = 0;
unsigned long lightsFade_end = 0;


void setup() {
  isMatrixLight = 0;
  Serial.begin(9600);
  Serial.print("Initializing");

  delay(100);
  ring.begin();
  Serial.print(".");
  delay(100);

  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, getColorByHueAndVal(map(i, 0, 15, hue_index_min, hue_index_max), 20));
    ring.show();
    Serial.print(".");
    delay(20);
  }
  Serial.print(".");
  delay(100);

  for (byte x = 0; x < num_single_buttons; x++) {
    pinMode(single_button_pins[x], INPUT);
    digitalWrite(single_button_pins[x], HIGH);
    Serial.print(".");
    delay(20);
  }
  Serial.print(".");
  delay(100);

  // Initialize spectrum analyzer
  pinMode(SPECTRUMSHIELD_PIN_RESET, OUTPUT); // reset
  pinMode(SPECTRUMSHIELD_PIN_STROBE, OUTPUT); // strobe
  digitalWrite(SPECTRUMSHIELD_PIN_RESET, LOW); // reset low
  digitalWrite(SPECTRUMSHIELD_PIN_STROBE, HIGH); //pin 5 is RESET on the shield

  Serial.print(".");
  delay(100);

  // Initialize rf24 radio
  radio.begin();
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(10, 15);
  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_16);

  Serial.print(".");

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.openReadingPipe(2, addresses[2]); // WORKING HERE
  radio.startListening();

  Serial.print(".");

  // Initialize Wii nunchuk controller
  nunchuk.init();

  delay(200); // Prevent buttons from being triggered while initializing
  Serial.print(".");

  Serial.println(" Ready!");
}

void loop() {
  if (receiveControlData()) {
    handleReceivedControlData();
  }

  if (isIntervalTimeReached()) {
    readSpectrum();
    processModeData();
    readNunchuk();
    processAllLightsFade();
    processSingleLightFade();
    interpretNunchukJoystick();
    interpretNunchukButtons();
    interpretNunchukAccel();
    interpretSingleButtons();
    interpretPots();
    transmitLightsEffectsData();
    //    printLightSettings();
  }
  updatePersistentTriggers(); // Buttons held down

}


/*
   Handle long button presses
*/
void updatePersistentTriggers() {
  if (persistent_trigger_active && millis() >= (persistent_trigger_last_updated + persistent_trigger_update_interval)) {
    persistent_trigger_last_updated = millis();
    switch (persistent_trigger_index) {
      case 0:
        setActiveDialSettingValue();
        break;
    }
  }
}


/*
   Process data received from external controllers
*/
void handleReceivedControlData() {
  Serial.println("handleReceivedControlData()");
  if (receivedControlData.changeHue) {
    Serial.println(receivedControlData.hue);
  }
  if (receivedControlData.changeBrightness) {
    Serial.println(receivedControlData.brightness);
  }
  if (receivedControlData.changeMode) {
    Serial.println(receivedControlData.mode);
  }
  if (receivedControlData.performAction) {
    Serial.println(receivedControlData.action);
  }
}


/*
   Update mode-driving variables
*/
void processModeData() {

  // Update position
  //  unsigned long currPosInterval = map(lights.pos, 0, 255, pos_change_interval_min, pos_change_interval_max);
  processModePositionData();

  if (isAutomaticColorCycleEnabled()) {
    // Update hue
    if (millis() >= (auto_cycle_hue_last_update + auto_cycle_hue_change_interval)) {
      auto_cycle_hue_last_update = millis();
      autoIterateHue();
    }
  }

  // Update strobe
  processStrobeData();
}


void processModePositionData() {

  unsigned long cycle_time_elapsed = millis() - position_cycle_time_started;

  //  if (cycle_time_elapsed >= getCycleLengthFromMainPot()) {
  if (cycle_time_elapsed >= position_cycle_time) {
    position_cycle_time_started = millis();
    if (pos_direction) {
      pos_direction = 0;
    } else {
      pos_direction = 1;
    }

  } else {
    if (pos_direction) {
      lights.pos = map(cycle_time_elapsed, 0, position_cycle_time, 0, 255);
    } else {
      lights.pos = map(cycle_time_elapsed, 0, position_cycle_time, 255, 0);
    }
  }

}

void processStrobeData() {
  unsigned long strobe_interval = lights.on ? strobe_light_duration : strobe_pause_duration;
  if (millis() >= (strobe_last_update + strobe_interval)) {
    strobe_last_update = millis();
    lights.on ? lights.on = 0 : lights.on = 1;
  }
}

/*
   Debug utility
*/
void printLightSettings() {
  Serial.print("Hue: "); Serial.print(lights.hue); Serial.print(", ");
  Serial.print("Pos: "); Serial.print(lights.pos); Serial.print(", ");
  Serial.print("Strobe: "); Serial.println(lights.on);
}


/*
   Transmit data via radio
*/
void transmitLightsEffectsData() {
  radio.stopListening();
  radio.openWritingPipe(addresses[0]);
  radio.write(&lights, sizeof(lights));
  radio.startListening();
}


/*
   Receive data via radio
*/
bool receiveControlData() {
  byte pipe_num = 1;
  radio.openReadingPipe(pipe_num, addresses[pipe_num]);
  radio.startListening();
  if (radio.available(&pipe_num)) {
    while (radio.available(&pipe_num)) {
      radio.read(&receivedControlData, sizeof(receivedControlData));
    }
    return true;
  } else {
    return false;
  }
}


/*
   Update events based on interval
*/
bool isIntervalTimeReached() {
  if (millis() >= (update_timestamp + update_interval)) {
    update_timestamp = millis();
    return true;
  } else {
    return false;
  }
}


/*
   Read audio data
*/
bool readSpectrum(void) {

  // Reset spectrum analysis data
  digitalWrite(SPECTRUMSHIELD_PIN_RESET, HIGH);
  digitalWrite(SPECTRUMSHIELD_PIN_RESET, LOW);

  // Loop thru audio bands
  for (int band = 0; band < 7; band++) {
    digitalWrite(SPECTRUMSHIELD_PIN_STROBE, LOW); // go to the next band
    delayMicroseconds(50); //gather some data
    left[band]  = getBandValue(analogRead(SPECTRUMSHIELD_PIN_LEFT)); // store left band reading
    right[band] = getBandValue(analogRead(SPECTRUMSHIELD_PIN_RIGHT)); // store right band reading
    lights.left[band]  = mapSoundDataForTransmission(left[band]);
    lights.right[band] = mapSoundDataForTransmission(right[band]);
    digitalWrite(SPECTRUMSHIELD_PIN_STROBE, HIGH); // reset the strobe pin
  }

  if (behavior_debug_output) {
    prettyPrintSoundData(lights.left, false);
    prettyPrintSoundData(lights.right, true);
  }
  return true;
}


/*
   Translate audio band data from ints into bytes for transmission
*/
byte mapSoundDataForTransmission(int rawValue) {
  return map(rawValue, 0, 1023, 0, 255);
}


/*
   Trim minimum values based on cut-off setting
*/
int getBandValue(int analogValue) {
  return (analogValue >= main_pot_audio_min) ? map(analogValue, main_pot_audio_min, 1023, 0, 1023) : 0;
}



/*
   Translate joystick into action areas for buttons

   Joystick position:

                Command 1
                    |
    Command 4 - Command 0 - Command 2
                    |
                Command 3
*/
void interpretNunchukJoystick() {
  byte pos_x = getJoystickPositionIndex(nunchuk.analogX);
  byte pos_y = getJoystickPositionIndex(nunchuk.analogY);


  if (pos_x == 1) { // Horizontally centered
    if (pos_y == 0) { // Up
      setJoystickPosition(3);
    } else if (pos_y == 2) { // Down
      setJoystickPosition(1);
    } else {
      setJoystickPosition(0); // Default
    }
  } else if (pos_y == 1) { // Vertically centered
    if (pos_x == 0) { // Left
      setJoystickPosition(4);
    } else if (pos_x == 2) { // Right
      setJoystickPosition(2);
    } else {
      setJoystickPosition(0); // Default
    }
  } else {
    setJoystickPosition(0); // Default
  }
}

/*
   Read potentiometers
    - General purpose (hue, brightness, speed, percent, etc.)
    - Audio min trim
*/
void interpretPots() {
  for (int pot = 0; pot < num_pots; pot++) {
    switch (pot) {
      case 0:
        pot_state[pot] = analogRead(POTENTIOMETER_PIN_HUE);
        break;
      case 1:
        pot_state[pot] = analogRead(POTENTIOMETER_PIN_AUDIO_MIN);
        break;
    }
    if (pot_state[pot] < 5) pot_state[pot] = 5;
    if (pot_state[pot] > 1023) pot_state[pot] = 1023;
  }

  // Set minimum audio trim
  main_pot_audio_min = map(pot_state[1], 0, 1023, audio_trim_min, audio_trim_max);

  //  main_pot_curr = map(pot_state[0], 0, 1023, hue_index_min, hue_index_max);

  //  if (main_pot_curr != main_pot) {
  if (pot_state[0] != pot_state_last[0]) {

    int stateDiff = pot_state[0] - pot_state_last[0];
    if (stateDiff < 0) {
      stateDiff = stateDiff * -1;
    }

    // Don't change if value has only changed by 1, potentiometer can drift enough to trigger it, lights up too much
    if (stateDiff > 1) {
      int newHue = getHueFromMainPot();
      //      main_pot = newHue;
      hue_change_last_update = millis();
      showActiveDialSetting();

    }
  }
}

int getHueFromMainPot() {
  return map(pot_state[0], 0, 1023, hue_index_min, hue_index_max);
}

int getBrightnessFromMainPot() {
  return map(pot_state[0], 0, 1023, 50, 255);
}

int getPercentFromMainPot() {
  return map(pot_state[0], 0, 1023, 10, 90);
}

int getCycleLengthFromMainPot() {
  return map(pot_state[0], 3, 1023, position_cycle_time_max, position_cycle_time_min);
}

/*
   Translate single buttons into actions
*/
void interpretSingleButtons() {
  for (int pos = 0; pos < num_single_buttons; pos++) {
    single_button_state[pos] = (digitalRead(single_button_pins[pos]) == LOW) ? 1 : 0;

    if (single_button_state[pos] != single_button_last_state[pos]) {
      single_button_last_debounce_time[pos] = millis();
    }

    if ((millis() - single_button_last_debounce_time[pos]) < debounce_delay) {
      if (single_button_state[pos] != single_button_curr_state[pos]) {
        single_button_curr_state[pos] = single_button_state[pos];
        if (single_button_curr_state[pos]) {
          triggerSingleButton(pos);
        }
      }
    }
  }
}

/*
   Translate wii buttons into actions
*/
void interpretNunchukButtons() {
  for (int pos = 0; pos < 2; pos++) {
    button_state[pos] = pos == 0 ? nunchuk.zButton : nunchuk.cButton;

    if (button_state[pos] != button_last_state[pos]) {
      button_last_debounce_time[pos] = millis();
    }

    if ((millis() - button_last_debounce_time[pos]) < debounce_delay) {
      if (button_state[pos] != button_curr_state[pos]) {
        button_curr_state[pos] = button_state[pos];
        if (button_curr_state[pos]) {
          onButtonDown(pos);
        } else {
          onButtonUp(pos);
        }
      }
    }
  }
}

void interpretNunchukAccel() {

  //  max_brightness = map(rawVal, 0, 140, 20, 100);

}

void setBrightnessToPitch() {
  int rawVal = nunchuk.pitch;
  if (rawVal > 140) rawVal = 140;
  lights.brightness = map(rawVal, 0, 140, 20, 100);
}


void triggerSingleButton(byte index) {
  Serial.println(index);
  changeEffectsMode(index + 1);
}


/*
   Nunchuk button has been pressed
*/
void onButtonDown(int isSmallButton) {
  pitch = nunchuk.pitch;

  if (nunchuk.roll < -90) {
    roll = 0;
  } else if (nunchuk.roll > 90) {
    roll = 255;
  } else {
    roll = map(nunchuk.roll, -90, 90, 0, 255);
  }

  // Small top button (c) - set mode
  if (isSmallButton) {
    changeSettingUpdatedByDial(joystickPositionIndex);
    //    changeEffectsMode(joystickPositionIndex);

    // Large bottom button (z) - perform action
  } else {

    triggerAction(joystickPositionIndex);
  }
}


/*
   Nunchuk button has been lifted
*/
void onButtonUp(int index) {
  if (persistent_trigger_active) {
    switch (persistent_trigger_index) {
      case 0:
        singleLightFadeOff(lights.hue, max_brightness, 500, getDialLightIndex(pot_state[0]));
        //        startAllLightsFadeOff(lights.hue, max_brightness, 500);
        break;
    }
    persistent_trigger_active = 0;
    persistent_trigger_index = 0;
  }
}


void changeEffectsMode(byte newMode) {
  lights.mode = newMode;
  Serial.print("Changing mode to "); Serial.println(lights.mode);
  //  if(lights.mode == 0) {
  //    clearAllLights();
  //  }
}


/*
   Which parameter is being changed by main dial
*/
void changeSettingUpdatedByDial(byte index) {
  main_pot_setting = index;
  switch (index) {
    case 0:
      clearAllLights();
      break;
    case 1: // Brightness

      break;
    case 2: // Speed

      break;
    case 3: // Hue

      break;
    case 4: // Percent

      break;
  }
}


/*
   Store dial setting relative to active setting
*/
void setActiveDialSettingValue() {
  switch (main_pot_setting) {
    case 0:
      //      clearAllLights();

      //      setAllLights(getColorByHueAndVal(lights.hue, max_brightness));
      break;
    case 1: // Brightness
      //      lights.brightness = getBrightnessFromMainPot();
      break;
    case 2: // Speed
      //      position_cycle_time = getCycleLengthFromMainPot();
      break;
    case 3: // Hue
      lights.hue = getHueFromMainPot();
      disableAutomaticColorCycle();

      break;
    case 4: // Percent
      //      lights.percent = getPercentFromMainPot();
      break;
  }

}


/*
   Display current dial settings
*/
void showActiveDialSetting() {
  switch (main_pot_setting) {
    case 0:
      break;
    case 1: // Brightness
      lights.brightness = getBrightnessFromMainPot();
      showBrightnessDialSetting();
      break;
    case 2: // Speed
      position_cycle_time = getCycleLengthFromMainPot();
      showSpeedDialSetting();
      break;
    case 3: // Hue
      showHueDialSetting();

      break;
    case 4: // Percent
      lights.percent = getPercentFromMainPot();
      showPercentDialSetting();
      break;
  }

}


void showHueDialSetting() {
  int activeIndex = getDialLightIndex(pot_state[0]); // Active light index relative to dial
  int tempHue = getHueFromMainPot();
  uint32_t color = getColorByHueAndVal(tempHue, max_brightness);
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, i == activeIndex ? color : BLACK);
  }
  ring.show();
}


void showBrightnessDialSetting() {
  int activeIndex = getDialLightIndex(pot_state[0]); // Active light index relative to dial
  int tempBrightness = map(pot_state[0], 0, 1023, brightness_setting_min, brightness_setting_max);
  uint32_t color = getColorByHueAndVal(lights.hue, tempBrightness);;
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, i == activeIndex ? color : BLACK);
  }
  ring.show();
}


void showSpeedDialSetting() {
  int activeIndex = getLightPosByDialIndex(map(lights.pos, 0, 255, 0, 12));
  uint32_t color = getColorByHueAndVal(lights.hue, max_brightness);
  for (int i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, i == activeIndex ? color : BLACK);
  }
  ring.show();
}


void showPercentDialSetting() {
  int activeIndex = getLightPosByDialIndex(map(lights.pos, 0, 255, 0, 12));
  uint32_t color = getColorByHueAndVal(lights.hue, max_brightness);
  uint32_t randColor;

  for (int i = 0; i < ring.numPixels(); i++) {
    randColor = random((1 / 16 * 100), 100) <= getPercentFromMainPot() ? color : BLACK;
    ring.setPixelColor(i, randColor);
  }
  ring.show();
}

/*
  Get position on light ring relative to dial
  There is a 'dead zone' of three pixels on the bottom
*/
byte getDialLightIndex(int val) {
  int pos = map(val, 0, 1010, 0, 12);

  return getLightPosByDialIndex(pos);
}

int getLightPosByDialIndex(int index) {
  if (index >= 0 && index <= 5) {
    return map(index, 0, 5, 5, 0);
  } else if (index >= 6 && index <= 12) {
    return map(index, 6, 12, 15, 9);
  }
}

/*
   Perform action
*/
void triggerAction(byte index) {
  Serial.print("Action "); Serial.println(joystickPositionIndex);
  Serial.print("Mode is "); Serial.println(lights.mode);

  if (index == 0) {
    persistent_trigger_active = 1;
    persistent_trigger_index = 0;
    return;
  }

  // Actions are specific to current mode
  switch (lights.mode) {
    case 0: // All on (?)
      triggerMainActions(index);
      break;
    case 1: // All pulse
      triggerMainActions(index);
      break;
    case 2: // All strobe
      triggerMainActions(index);
      break;
    case 3: // All sparkle
      triggerMainActions(index);
      break;
    case 4: // Audio reactive
      triggerMainActions(index);
      break;
  }
}

/*
   Trigger main actions
*/
void triggerMainActions(byte index) {
  switch (index) {
    case 0:
      disableAutomaticColorCycle();
      //      lights.hue = roll;
      Serial.print("Setting hue to "); Serial.println(roll);

      break;
    case 1:
      setBrightnessToPitch();
      Serial.print("Setting brightness to "); Serial.println(lights.brightness);
      break;
    case 2:

      break;
    case 3:
      enableAutomaticColorCycle();
      break;
    case 4:

      break;
  }
}

/*
   Trigger main actions
*/
void triggerAudioActions(byte index) {
  switch (index) {
    case 0:

      break;
    case 1:

      break;
    case 2:

      break;
    case 3:

      break;
    case 4:

      break;
  }
}


/*
   Set active flag and joystick position
*/
void setJoystickPosition(byte pos) {
  joystickPositionIndex = pos;

  // Color is still fading from a previous change
  if (millis() <= hue_change_last_update + hue_change_display_time) {

    // Joystick is centered
  } else if (pos == 0) {
    if (!persistent_trigger_active && !isAllLightsFadeActive()) {
      clearAllLights();
    }

  } else if (!persistent_trigger_active) {
    //    setActiveJoystickPositionLED();
    //    showActiveDialSetting();
  }
}

/*
   Show four positions in ring with colors relative to button(s) pressed
*/
void setActiveJoystickPositionLED() {
  uint32_t color;
  if (!nunchuk.zButton && !nunchuk.cButton) {
    color = RED;
  } else if (nunchuk.zButton && !nunchuk.cButton) {
    color = BLUE;
  } else if (!nunchuk.zButton && nunchuk.cButton) {
    color = GREEN;
  } else {
    color = PURPLE;
  }
  setSingleLight(15 - ((joystickPositionIndex - 1)) * 4, color);
}


/*
   Map joystick regions into 0, 1, 2
*/
byte getJoystickPositionIndex(int val) {
  if (val < joystickOffset) {
    return 0;
  } else if (val > (255 - joystickOffset)) {
    return 2;
  } else {
    return 1;
  }
}

/*
   Read Wii Nunchuk data
*/
void readNunchuk() {
  nunchuk.update();    //try to update local nunchuk variables from the nunchuk

  if (false) {
    Serial.print(" X Axis:" );
    Serial.print(nunchuk.analogX, DEC);
    Serial.print(", Y Axis:" );
    Serial.println(nunchuk.analogY, DEC);
  }

  if (behavior_debug_output) {
    //print all nunchuk data
    Serial.print(" X Axis:" );
    Serial.print(nunchuk.analogX, DEC);
    Serial.print(" Y Axis:" );
    Serial.print(nunchuk.analogY, DEC);
    Serial.print(" Acc X:" );
    Serial.print(nunchuk.accelX, DEC);
    Serial.print(" Acc Y:" );
    Serial.print(nunchuk.accelY, DEC);
    Serial.print(" Acc Z:" );
    Serial.print(nunchuk.accelZ, DEC);
    Serial.print(" Pitch:" );
    Serial.print(nunchuk.pitch, DEC);
    Serial.print(" Roll:" );
    Serial.print(nunchuk.roll, DEC);
    Serial.print(" Z button:" );
    Serial.print(nunchuk.zButton, DEC);
    Serial.print(" C button:" );
    Serial.println(nunchuk.cButton, DEC);
  }
}


/*
   Clear pixels
*/
void clearAllLights() {
  for (uint16_t i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, BLACK);
  }
  ring.show();
}


/*
   Set all pixels to single color
*/
void setAllLights(uint32_t color) {
  for (uint16_t i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, color);
  }
  ring.show();
}


/*
   Set single pixel in ring, set others to black
*/
void setSingleLight(uint16_t lightIndex, uint32_t color) {
  for (uint16_t i = 0; i < ring.numPixels(); i++) {
    ring.setPixelColor(i, lightIndex == i ? color : BLACK);
  }
  ring.show();
}


/*
  Get RGB light variable from HSV values
*/
//uint32_t getColorByHueAndVal(int hue, int val) {
//  unsigned char red, green, blue;
//  int sat = 255;
//  //  int max_brightness = 255;
//
//  hsv2rgb(hue, sat, val, &red, &green, &blue, max_brightness);
//
//  return ring.Color(red, green, blue);
//}


/*
   Process fading lights over specified time
*/
void singleLightFadeOn(int hue, byte val, unsigned long fadeLength, int index) {
  Serial.println("startAllLightsFadeOn()");
  lightsFade_active = 1;
  lightsFade_direction = 1;
  lightsFade_fadeAll = 0;
  lightsFade_lightIndex = index;
  lightsFade_hue = hue;
  lightsFade_val = val; // Ending val to fade to
  lightsFade_start = millis();
  lightsFade_end = lightsFade_start + fadeLength;
}


/*
   Process fading lights over specified time
*/
void singleLightFadeOff(int hue, byte val, unsigned long fadeLength, int index) {
  Serial.println("singleLightFadeOff()");
  lightsFade_active = 1;
  lightsFade_direction = 0;
  lightsFade_fadeAll = 0;
  lightsFade_lightIndex = index;
  lightsFade_hue = hue;
  lightsFade_val = val; // Ending val to fade to
  lightsFade_start = millis();
  lightsFade_end = lightsFade_start + fadeLength;
}


/*
   Process fading lights over specified time
*/
void startAllLightsFadeOn(int hue, byte val, unsigned long fadeLength) {
  Serial.println("startAllLightsFadeOn()");
  lightsFade_active = 1;
  lightsFade_direction = 1;
  lightsFade_fadeAll = 1;
  lightsFade_hue = hue;
  lightsFade_val = val; // Ending val to fade to
  lightsFade_start = millis();
  lightsFade_end = lightsFade_start + fadeLength;
}


/*
   Process fading lights over specified time
*/
void startAllLightsFadeOff(int hue, byte val, unsigned long fadeLength) {
  Serial.println("startAllLightsFadeOff()");
  lightsFade_active = 1;
  lightsFade_direction = 0;
  lightsFade_fadeAll = 1;
  lightsFade_hue = hue;
  lightsFade_val = val; // Starting val to fade from
  lightsFade_start = millis();
  lightsFade_end = lightsFade_start + fadeLength;
}


/*
   Stop fading lights
*/
void cancelLightsFade() {
  lightsFade_active = 0;
}


/*
   Is a fade action current
*/
byte isAllLightsFadeActive() {
  return lightsFade_active && lightsFade_fadeAll;
}


byte isSingleLightsFadeActive() {
  return lightsFade_active && !lightsFade_fadeAll;
}


/*
   Update single fading lights progress
*/
void processSingleLightFade() {
  if (persistent_trigger_active) return; // A button is currently held down, don't process fade
  if (isSingleLightsFadeActive()) {
    if (millis() >= lightsFade_end) { // Calculated time for fade has elapsed
      cancelLightsFade();
      if (lightsFade_direction) { // Completed fadeOn

      } else {
        clearAllLights(); // Completed fadeOff
      }
      return;
    }

    setSingleLight(lightsFade_lightIndex, getColorByHueAndVal(lightsFade_hue, getColorValByFadeTiming())); // Update fade
  }
}


/*
   Update fading lights progress
*/
void processAllLightsFade() {
  if (persistent_trigger_active) return; // A button is currently held down, don't process fade
  if (isAllLightsFadeActive()) {
    if (millis() >= lightsFade_end) { // Calculated time for fade has elapsed
      cancelLightsFade();
      if (lightsFade_direction) { // Completed fadeOn

      } else {
        clearAllLights(); // Completed fadeOff
      }
      return;
    }
    setAllLights(getColorByHueAndVal(lightsFade_hue, getColorValByFadeTiming())); // Update fade
  }
}


/*
   Calculate color val based on progress through fade up/down
*/
byte getColorValByFadeTiming() {
  return map(
           millis(),
           lightsFade_start,
           lightsFade_end,
           lightsFade_direction ? 0 : lightsFade_val,
           lightsFade_direction ? lightsFade_val : 0
         );
}

/*
   Color cycling utilities
*/
bool isAutomaticColorCycleEnabled() {
  return behavior_auto_cycle_hue;
}

void enableAutomaticColorCycle() {
  behavior_auto_cycle_hue = true;
}

void disableAutomaticColorCycle() {
  behavior_auto_cycle_hue = false;
}

void autoIterateHue() {
  lights.hue++;
  if (lights.hue > hue_index_max) {
    lights.hue = hue_index_min;
  }
}

