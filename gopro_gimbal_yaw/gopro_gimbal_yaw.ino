#include <Servo.h>

#define YAW_OFFSET 0

// Make sure to connect pitch channel (PITCH_RX) to pin 3 on arduino (pwm)
// and roll channel (ROLL_RX) to pin 5 on arduino (pwm)
// And dont forget to connect arduino GROUND to basecam GROUND to create the reference point

Servo servoHeading;
Servo servoPitch;

float fmap(float x, float in_min, float in_max, float out_min, float out_max);
void setPitch(float value);
void setHeading(float value);


void setup() {
  Serial.begin(115200);
  servoHeading.attach(3);
  servoPitch.attach(5);
  delay(500);
  setHeading(YAW_OFFSET); // add current heading angle
  setPitch(0);
  delay(500);
  //servoPitch.write(120);
  //servoHeading.write(150);
  Serial.println("Hi, I'm the camera driver.");
  digitalWrite(LED_BUILTIN, LOW);
}


int counter = 0;
char inData[64]; // Allocate some space for the string
char packetID[7];


void loop() {
  if (Serial.available())
  {
    char recieved = Serial.read(); // read next char
    inData[counter++] = recieved; // add it to the string
    if (recieved == '\n') // if the end of the string is recieved
    {
      strncpy(packetID, inData, 6); // copy first 6 chars to packetID array
      packetID[6] = '\0'; // and add EOL symbol to it
      if (strcmp(packetID, "campos") == 0) // check if packedID matched camera position message
      {
        // The message should be [campos heading pitch] (for example [campos 30 -19]). Now we need to extract the numeric values from the string
        char * pch;
        char buf[64];
        pch = strtok(inData, " ");
        strcpy(buf, pch); // copy it
        int cnt = 0;
        int heading = 0;
        int pitch = 0;
        while (pch != NULL)
        {
          if (cnt == 1)
            heading = atoi(pch);
          if (cnt == 2)
            pitch = atoi(pch);
          pch = strtok(NULL, " "); // this continues where the previous call left off
          cnt++;
        }
        Serial.print("New position ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.println(pitch);
        setHeading(YAW_OFFSET + heading);
        setPitch(pitch);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        // --- Got the above part from the web, it looks wierd, but it works!
      }
      memset(inData, 0, sizeof(inData)); // Clear recieved buffer
      counter = 0;
    } // if (recieved == '\n')
  } // if (Serial.available())
} // loop


float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Positive - up(back); negative - down(front)
void setPitch(float value)
{
  if (value > 21)
    value = 21;
  if (value < -45)
    value = -45;
  int pwm = fmap(value, -45, 21, 56, 110);
  servoPitch.write(pwm);
}

// Positive - Clockwize; negative - Countercloclwise
// dont forget to add current heading angle
void setHeading(float value)
{
  if (value > 120)
    value = 120;
  if (value < -120)
    value = -120;
  int pwm = fmap(value, -90, 90, 34, 150);
  servoHeading.write(pwm);
}
