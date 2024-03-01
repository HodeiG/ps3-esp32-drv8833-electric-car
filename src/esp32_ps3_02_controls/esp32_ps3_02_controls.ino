#include <Ps3Controller.h>

/*
  GPIO = General Purpose Input/Output. This means you can program this pin to be either
  an input (reading digital signals from sensors) or an output (sending digital signals
  to control devices like LEDs).

  ESP32 = Chip microcontroler with WiFi and Blueetooth

  DRV8833 = Motor driver. It allows to control 2 motors

  PWM = Pulse With Modulation. The ESP32-WROOM-32D supports 16 independent PWM channels.
  This means you can use any of the GPIO pins except 34, 35, 36, 37, 38, and 39 for PWM
  output. These last six pins are dedicated to other functions and lack the necessary
  hardware for PWM generation.
*/


/*
   EEP2: Set the ESP32 into sleep mode or active mode.
         By default, it will be set to HIGH, so it's always active
*/
const int DRV8833_EEP = 2;  // Sleep mode input, active high
/*
   LEFT MOTORS:
      IN1: ESP32:GPIO-14 will be used to connect with DRV8833:GPIO-IN1
      IN2: ESP32:GPIO-12 will be used to connect with DRV8833:GPIO-IN1
   RIGHT MOTORS:
      IN3: ESP32:GPIO-27 will be used to connect with DRV8833:GPIO-IN1
      IN4: ESP32:GPIO-25 will be used to connect with DRV8833:GPIO-IN1
*/
const int DRV8833_IN1 = 14;
const int DRV8833_IN2 = 12;
const int DRV8833_IN3 = 27;
const int DRV8833_IN4 = 25;


// Minimum PWM value required to start or stop the motor.
const int MINIMUM_PWM_START_STOP = 10;

// Additional PWM boost applied to ensure reliable motor starting.
// Experimentation has shown that a PWM value around 110-1300 reliably
// initiates motor motion. This constant ensures that even if the calculated
// PWM falls slightly below this range, the motor still starts reliably.
const int PWM_BOOST_FOR_START = 130;

// Maximum PWM value allowed by the analogWrite function.
// This constant ensures that the combined PWM value (including the boost)
// does not exceed the maximum output value supported by the function (255).
const int MAX_ANALOG_WRITE_VALUE = 255;


enum CarSide {LEFT_SIDE, RIGHT_SIDE};
enum MovementDirection {FORWARD, REVERSE};

/*
 * Enhancement 1: Addressing intermittent motion issue due to rapid PS3 events.
 * 
 * Observation: Sometimes, when a button was released (e.g., cross button),
 * the car would continue moving due to frequent PS3 events being sent.
 * 
 * Analysis: PS3 events are sent approximately every 10-15 milliseconds,
 * leading to unnecessary speed change requests.
 * 
 * Solution: Record the last event type and value, and initiate a speed change
 * only if there's a significant value change, determined by the event_value_threshold.
 * Ensure the margin is not larger than MINIMUM_PWM_VALUE to prevent motors from not stopping.
 */
enum PS3EventType  {NONE, CROSS, SQUARE, STICK};
PS3EventType  last_PS3_event_type   = NONE;
int last_event_value = 0;
int event_value_threshold = 10;


void control(CarSide side, MovementDirection m_direction, int m_speed = 0) {
  int pwm_pin;
  int low_pin;
  if (side == LEFT_SIDE) {
    if (m_direction == FORWARD) {
      pwm_pin = DRV8833_IN1;
      low_pin = DRV8833_IN2;
    } else {
      pwm_pin = DRV8833_IN2;
      low_pin = DRV8833_IN1;
    }
  } else {
    if (m_direction == FORWARD) {
      pwm_pin = DRV8833_IN3;
      low_pin = DRV8833_IN4;
    } else {
      pwm_pin = DRV8833_IN4;
      low_pin = DRV8833_IN3;
    }
  }
  if (m_speed > MINIMUM_PWM_START_STOP) {
    analogWrite(pwm_pin, min(MAX_ANALOG_WRITE_VALUE, m_speed + PWM_BOOST_FOR_START));
    analogWrite(low_pin, LOW);
  } else { // Stop
    analogWrite(pwm_pin, LOW);
    analogWrite(low_pin, LOW);
  }
}

void notify()
{
  int button_pressure = 0;

  // Cross (X) button pressed
  if ( abs(Ps3.event.analog_changed.button.cross) ) {
    button_pressure = Ps3.data.analog.button.cross;
    if (last_PS3_event_type == CROSS &&
        button_pressure <= last_event_value + event_value_threshold &&
        button_pressure >= last_event_value - event_value_threshold) {
      return;
    } 
    last_PS3_event_type = CROSS;
    last_event_value = button_pressure;

    Serial.print("Pressing the cross button: ");
    Serial.println(button_pressure, DEC);
    control(LEFT_SIDE, FORWARD, button_pressure);
    control(RIGHT_SIDE, FORWARD, button_pressure);
  }

  // Square (■) button pressed
  if ( abs(Ps3.event.analog_changed.button.square) ) {
    button_pressure = Ps3.data.analog.button.square;
    if (last_PS3_event_type == SQUARE &&
        button_pressure <= last_event_value + event_value_threshold &&
        button_pressure >= last_event_value - event_value_threshold) {
      return;
    }
    last_PS3_event_type = SQUARE;
    last_event_value = button_pressure;
    
    Serial.print("Pressing the square button: ");
    Serial.println(button_pressure, DEC);
    control(LEFT_SIDE, REVERSE, button_pressure);
    control(RIGHT_SIDE, REVERSE, button_pressure);
  }

  // Left analog stick pressed
  if ( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ) {
    int x = Ps3.data.analog.stick.lx;
    int y = Ps3.data.analog.stick.ly;
    button_pressure = abs(x);
    if (last_PS3_event_type == STICK &&
        x <= last_event_value + event_value_threshold &&
        x >= last_event_value - event_value_threshold) {
      return;
    }
    last_PS3_event_type = STICK;
    last_event_value = x;
    
    Serial.print("Moved the left stick:");
    Serial.print(" x="); Serial.print(x, DEC);
    Serial.print(" y="); Serial.print(y, DEC);
    Serial.println();
    
    if (x < 0) {
      control(LEFT_SIDE, REVERSE, button_pressure);
      control(RIGHT_SIDE, FORWARD, button_pressure);
    } else {
      control(LEFT_SIDE, FORWARD, button_pressure);
      control(RIGHT_SIDE, REVERSE, button_pressure);
    }
  }
}

void onConnect() {
  Serial.println("Connected.");
}

void setup()
{
  Serial.begin(9600);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin();
  pinMode(DRV8833_EEP, OUTPUT);
  pinMode(DRV8833_IN1, OUTPUT);
  pinMode(DRV8833_IN2, OUTPUT);
  pinMode(DRV8833_IN3, OUTPUT);
  pinMode(DRV8833_IN4, OUTPUT);
}

void loop()
{
  digitalWrite(DRV8833_EEP, HIGH);
  if (!Ps3.isConnected())
    return;

  delay(1000);
}
