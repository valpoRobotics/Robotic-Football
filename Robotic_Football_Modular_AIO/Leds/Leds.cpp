#define RED_LED         A0          //Red LED control is wired to pin A0
#define GREEN_LED       A1          //Green LED control is wired to pin A1
#define BLUE_LED        A2          //Blue LED control is wired to pin A2

/**
 * Sets the LEDs to the correct output pins
 */
void ledsSetup() {
	pinMode(BLUE_LED,  OUTPUT);
	pinMode(GREEN_LED, OUTPUT);
	pinMode(RED_LED,   OUTPUT);
}

/**
 * Flashes the LEDs in the patten; green, blue, red, green, blue
 */
void flashLeds() {
  //flash the leds
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  delay(300);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
}

/**
 * Turns all the LEDs red
 */
void red() {
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
}

/**
 * Turns all the LEDs green
 */
void green() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
}

/**
 * Turns all the LEDs blue
 */
void blue() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
}