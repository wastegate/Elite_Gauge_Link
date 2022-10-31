#include <SPI.h>                                              // Harware SPI library
#include <mcp_can.h>                                          // CAN Library
#include <Adafruit_GFX.h>                                     // OLED GFX Library
#include <Adafruit_SSD1306.h>                                 // 1.3 inch OLED library

#include <EEPROM.h>                                           // EEPROM library

const byte led = 23;                                           // On board LED 23 will use for CAN connection indicator

//2.4 v2.0 OLED PINS H/W SPI                                   // Define OLED Pins
#define OLED_CS     11
#define OLED_RESET  10
#define OLED_DC     9
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

//2.4 OLED PINS S/W SPI                                       // Uncomment for 2.4 inch OLED
//#define OLED_MOSI   11
//#define OLED_CLK   10
//#define OLED_DC    9
//#define OLED_CS    12
//#define OLED_RESET 8
//Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//BUTTON CONFIG
const byte button = 4;
long buttonTimer = 0;
long longPressTime = 750;
boolean buttonActive = false;
boolean longPressActive = false;

//EEPROM CONFIG
byte addr = 1;                                                 // Initislise the EEPROM address
byte buttonPushCounter = EEPROM.read(1);                      // Counter for the number of button presses (get value from EEPROM)

//byte buttonPushCounter = 0;

//CAN CONFIG
unsigned char rxLen = 0;                                      // CAN message rx length
unsigned char rxBuf[8];                                       // CAN message number of bytes
MCP_CAN CAN0(17);                                             // Set CAN C/S pin
byte CANfilterset = 0;                                        // Check if the fitler is set

//ECU VALUES
int VALUE1 = 0;                                                  // VALUE1 integer
int VALUE2 = 0;                                                  // VALUE2 integer
int VALUE3 = 0;                                                  // VALUE1 integer
int VALUE4 = 0;                                                  // VALUE2 integer
word STATUS1 = 0;
int STATUS2 = 0;
int STATUS3 = 0;
int STATUS4 = 0;
int STATUS5 = 0;
int LIMIT1 = 0;
int CLIMIT1;
float FVALUE1 = 0;                                               // VALUE1 floating point (decemal place numbers)
float FVALUE2 = 0;                                               // VALUE2 floating point (decemal place numbers)
float FVALUE3 = 0;
float FVALUE4 = 0;
unsigned long line = 0;
unsigned long ID = 0;

//SCREEN 1 HEADING, DATA AND MAX/MIN VALUES
int MAX1 = -10000;
int MAX2 = -10000;
int MIN1 = 10000;
int MIN2 = 10000;
float FMAX1 = -10000;
float FMAX2 = -10000;
float FMIN1 = 10000;
float FMIN2 = 10000;

int ScreenOff = 0;

void setup() {

  Serial.begin(115200);                                       // Serial Comms speed

  pinMode(led, OUTPUT);                                       // Setup Pin 23 as Output Led

  if (buttonPushCounter >= 13)                                 // On first run EEPROM could be any value 0-254
  { EEPROM.write(1, 0);                                        // If it is above 13, set it to zero to agree with the 'if' loop
  }

  display.begin(SSD1306_SWITCHCAPVCC);                        // Initialise OLED
  display.setTextWrap(false);

  pinMode(button, INPUT_PULLUP);                              //Button config
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  display.setRotation(0);                                     // Set rotation (0) Normal, (2) 180 deg

  display.clearDisplay();
  display.setTextSize(3);                                     // Splash screen
  display.setTextColor(WHITE);
  display.setCursor(20, 5);
  display.println("ELITE");
  display.setCursor(20, 35);
  display.println("GAUGE G4x");

  display.display();
  delay(1500);

  display.clearDisplay();

  while (CAN_OK != CAN0.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ))                     // initiate CAN bus : baudrate = 1000k for Haltech
  {
    Serial.println(F("CAN BUS Shield init fail"));               // :(
    Serial.println(F(" Init CAN BUS Shield again"));             // Yes please
    delay(1000);
  }
  Serial.println(F("CAN BUS Shield init ok!"));                  // YAY!
  CAN0.setMode(MCP_NORMAL);

}

void drawRightString(const char *buf, int x, int y)
{
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, x, y, &x1, &y1, &w, &h); //calc width of new string
    display.setCursor(x - w / 2, y);
    display.print(buf);
}

void loop() 
{



  if (digitalRead(button) == LOW) {

    if (buttonActive == false) {

      buttonActive = true;
      buttonTimer = millis();

    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {

      // Clear the peak values

      longPressActive = true;

      CANfilterset = 0;

      resetValues();

      resetMaxMin();

      delay(50);
    }

  } else {

    if (buttonActive == true) {

      if (longPressActive == true) {

        longPressActive = false;

      } else {

        //INCREMENT THE BUTTON COUNTER AND SET THE FILTER
        buttonPushCounter++;

        if (buttonPushCounter >= 13) buttonPushCounter = 0; // Reset button counter to loop through the displays

        EEPROM.write(1, buttonPushCounter);               // Update the counter into EEPROM (address location 1)

        CANfilterset = 0;                                  // Reset the CAN filter for next CAN ID

        ScreenOff = 0;

        resetValues();

        resetMaxMin();

        display.clearDisplay();

        delay(50);

      }

      buttonActive = false;

    }
    display.clearDisplay();

// RPM and TPS start
    if (buttonPushCounter == 0) {                                      // buttonPushCounter is used to cylce through screens
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);                                 // CAN Mask (allow all messages to be checked)
        CAN0.init_Filt(0, 0, 0x00FA);                                 // Only allow CAN ID 0x00FA to the buffer

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX
        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf
                         

        if (rxBuf[0] == 0) {                                        // If CAN ID is 250 pull the following buffer info

          VALUE1 = word(rxBuf[3], rxBuf[2]);                          // First two of bytes ID250 are RPM (buffer 2,3), skip first byte as this is address for frame
          VALUE1 = ((VALUE1 / 10) * 10);                              // Divide by 10 and times by 10 to remove last digit
        }
          if (rxBuf[0] == 1) {                                        // Frame 2
          VALUE2 = word(rxBuf[5], rxBuf[4]);                          // Bytes 4 and 5 are TPS
          VALUE2 = (VALUE2 / 10);
                              }
        

        if (VALUE1 > MAX1) {
          MAX1 = VALUE1;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("RPM");

        display.setTextSize(4);
        display.setCursor(5, 12);
        if (VALUE1 < 10000) {
          display.print(" ");
        }
        if (VALUE1 < 1000) {
          display.print(" ");
        }
        if (VALUE1 < 100) {
          display.print(" ");
        }
        if (VALUE1 < 10) {
          display.print(" ");
        }
        display.println(VALUE1, DEC);                               // Show value1 (RPM) as a whole decimal number

        display.setTextSize(1);
        display.setCursor(0, 47);
        display.println("TPS");
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 47);
        display.print(VALUE2, DEC);
        display.setCursor(88, 57);
        display.print(MAX1, DEC);

        display.display();

        delay(50);

      }
    }


// MGP Start
    if (buttonPushCounter == 1) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX
        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 0) {                                      // If CAN ID is 360 pull the following buffer info, frame ID 1

          VALUE1 = word(rxBuf[7], rxBuf[6]);
          //VALUE1 = (VALUE1 * 10);
          FVALUE1 = ((VALUE1 / 6.894) - 14.6);
        }


        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("MGP PSI");

        display.setTextSize(4);
        display.setCursor(5, 15);
        if (FVALUE1 >= 0.0) {
          display.print("+");
        }
        display.print(FVALUE1, 1);

        display.setTextSize(1);
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 57);
        display.print(FMAX1, 1);
        display.display();


      }
    }


// Ethanol Content Start
    if (buttonPushCounter == 2) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX
        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

                                                                      // Get Ethanol %
        if (rxBuf[0] == 13) {                  // frame 14
          VALUE1 = word(rxBuf[5], rxBuf[4]);                                      
        }
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("ETHANOL %");
        display.setTextSize(5);
        display.setCursor(40, 20);
        if (VALUE1 < 10) {
          display.print(" ");
        }
        display.println(VALUE1, DEC);
        display.display();

      }
    }

// Coolent and Air Temps Start
    if (buttonPushCounter == 3) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX
        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf
        if (rxBuf[0] == 2) {

          VALUE1 = word(rxBuf[7], rxBuf[6]);
          FVALUE1 = (VALUE1 - 50.0);
        }
          if (rxBuf[0] == 3) {
          VALUE2 = word(rxBuf[3], rxBuf[2]);
          FVALUE2 = (VALUE2 - 50.0); 
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }
        if (FVALUE2 > FMAX2) {
          FMAX2 = FVALUE2;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("CTS");
        display.setCursor(0, 23);
        display.println("IAT");
        display.setTextSize(3);
        display.setCursor(40, 0);
        if (FVALUE1 < -100.0) {
          display.print(" ----- ");
        }
        display.println(FVALUE1, 1);
        display.setCursor(40, 23);
        if (FVALUE2 < -100.0) {
          display.print(" ----- ");
        }
        display.println(FVALUE2, 1);
        display.setTextSize(1);
        display.setCursor(0, 47);
        display.println("PEAK CTS");
        display.setCursor(88, 47);
        display.print(FMAX1, 1);
        display.setCursor(0, 57);
        display.println("PEAK IAT");
        display.setCursor(88, 57);
        display.print(FMAX2, 1);
        display.display();

      }
    }

// AFR Start
    if (buttonPushCounter == 4) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);


        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX
        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf


        if (rxBuf[0] == 6) {

          VALUE1 = word(rxBuf[5], rxBuf[4]);
          FVALUE1 = (((VALUE1 / 10.0) * 14.7) / 100.0);
        }


        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("AFR");

        display.setTextSize(5);
        display.setCursor(5, 15);
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        display.println(FVALUE1, 1);

        display.display();

      }
    }
// Oil Pressure Start
    if (buttonPushCounter == 5) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 8) {

          VALUE1 = word(rxBuf[5], rxBuf[4]);
          VALUE1 = ((VALUE1 / 10.0) - 100);
          FVALUE1 = ((VALUE1 * 14.5) / 100.0);
        }

        if (FVALUE1 < FMIN1) {
          FMIN1 = FVALUE1;
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("OIL P");
        display.setTextSize(4);
        display.setCursor(5, 12);
        if (FVALUE1 < 100.0) {
          display.print(" ");
        }
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        display.println(FVALUE1, 1);
        display.setTextSize(1);
        display.setCursor(0, 47);
        display.println("MIN");
        display.setCursor(88, 47);
        display.print(FMIN1, 1);
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 57);
        display.print(FMAX1, 1);
        display.display();

      }
    }
// Battery Voltage
    if (buttonPushCounter == 6) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 3) {

          VALUE1 = word(rxBuf[5], rxBuf[4]);
          FVALUE1 = (VALUE1 / 100.0);
        }

        if (FVALUE1 < FMIN1) {
          FMIN1 = FVALUE1;
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("BATT V");

        display.setTextSize(4);
        display.setCursor(25, 12);
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        display.println(FVALUE1, 1);

        display.setTextSize(1);
        display.setCursor(0, 47);
        display.println("MIN");
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 47);
        display.print(FMIN1, 1);
        display.setCursor(88, 57);
        display.print(FMAX1, 1);
        display.display();

      }
    }

// Gear and Speed Start
    if (buttonPushCounter == 7) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 4)  {

          VALUE1 = word(rxBuf[3],rxBuf[2]);
          if (rxBuf[0] == 9); {
          VALUE2 = word(rxBuf[3], rxBuf[2]);
          VALUE2 = (VALUE2 / 10);
                              }
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("GEAR");
        display.setCursor(0, 40);
        display.println("SPEED");
        display.setTextSize(5);
        display.setCursor(80, 0);
        display.println(VALUE1, DEC);
        display.setTextSize(3);
        display.setCursor(52, 40);
        if (VALUE2 < 100) {
          display.print(" ");
        }
        if (VALUE2 < 10) {
          display.print(" ");
        }
        display.println(VALUE2, DEC);
        display.display();

      }
    }
// Injector Duty Start
    if (buttonPushCounter == 8) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf


        if (rxBuf[0] == 1) {

          VALUE1 = word(rxBuf[7], rxBuf[6]);
          FVALUE1 = (VALUE1 / 10.0);
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }


        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("INJ %");

        display.setTextSize(4);
        display.setCursor(5, 12);
        if (FVALUE1 < 100.0) {
          display.print(" ");
        }
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        display.println(FVALUE1, 1);

        display.setTextSize(1);
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 57);
        display.print(FMAX1, 1);
        display.display();

      }
    }
// Fuel Pressure Start
    if (buttonPushCounter == 9) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);


        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf
        
        if (rxBuf[0] == 7) {

          VALUE1 = word(rxBuf[7], rxBuf[6]);
          FVALUE1 = ((VALUE1 / 6.895) * 1.0);
        }

        if (FVALUE1 < FMIN1) {
          FMIN1 = FVALUE1;
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("FUEL P");

        display.setTextSize(4);
        display.setCursor(5, 12);
        if (FVALUE1 < 100.0) {
          display.print(" ");
        }
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        display.println(FVALUE1, 1);

        display.setTextSize(1);
        display.setCursor(0, 47);
        display.println("MIN");
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 47);
        display.print(FMIN1, 1);
        display.setCursor(88, 57);
        display.print(FMAX1, 1);

        display.display();

      }
    }

// Oil Temp Start
    if (buttonPushCounter == 10) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf


        if (rxBuf[0] == 8) {

          VALUE1 = word(rxBuf[3], rxBuf[2]);
          FVALUE1 = (VALUE1 - 50.0);
        }

        if (FVALUE1 > FMAX1) {
          FMAX1 = FVALUE1;
        }


        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("OIL T");

        display.setTextSize(4);
        display.setCursor(8, 12);
        if (FVALUE1 < 100.0) {
          display.print(" ");
        }
        if (FVALUE1 < 10.0) {
          display.print(" ");
        }
        if (FVALUE1 < -100.0) {
          display.setCursor(8, 12);
          display.print(" --- ");
        }
        display.println(FVALUE1, 1);
        display.setTextSize(1);
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 57);
        display.print(FMAX1, 1);
        display.display();

      }
    }
// Ignition Timing Start
    if (buttonPushCounter == 11) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 4) {
          VALUE1 = word(rxBuf[7], rxBuf[6]);
          FVALUE1 = ((VALUE1 * 0.1) - 100);
        }

        if (FVALUE1 > MAX1) {
          MAX1 = FVALUE1;
        }


        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("IGN Timing");

        display.setTextSize(5);
        display.setCursor(5, 15);
        if (FVALUE1 < 1000) {
          display.print(" ");
        }
        if (FVALUE1 < 100) {
          display.print(" ");
        }
        if (FVALUE1 < 10) {
          display.print(" ");
        }

        if (FVALUE1 < -100) {
          display.setCursor(5, 15);
          display.print(" --- ");
        }
        display.println(FVALUE1, DEC);

        display.setTextSize(1);
        display.setCursor(0, 57);
        display.println("MAX");
        display.setCursor(88, 57);
        display.print(MAX1, DEC);
        display.display();

      }
    }

/* Cutting due to space limits
// Cam Angle Start
    if (buttonPushCounter == 12) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf


        if ((canId == 0x00FA) && (rxBuf[0] == 5))  {

          VALUE1 = word(rxBuf[3],rxBuf[2]);                 //Cam Inlet Bank 1
          FVALUE1 = (VALUE1 * 0.1);
          VALUE2 = word(rxBuf[5],rxBuf[4]);                 //Cam Inlet Bank 2
          FVALUE2 = (VALUE2 * 0.1);
          VALUE3 = word(rxBuf[7],rxBuf[6]);                 //Cam Exhaust Bank 1
          FVALUE3 = (VALUE3 * 0.1);
          }
        if ((canId == 0x00FA) && (rxBuf[0] == 6)) {
          VALUE4 = word(rxBuf[3], rxBuf[2]);                //Cam Exhaust Bank 2
          FVALUE4 = (VALUE4 * 0.1);
          }
        }

        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Cam Angle");
        display.setCursor(0, 10);
        display.println("Inlet 1:");
        display.setCursor(70, 10);
        display.print(FVALUE1, DEC);
        display.setCursor(0, 20);
        display.print("Inlet 2: ");
        display.setCursor(70, 20);
        display.print(FVALUE2, DEC);
        display.setCursor(0, 30);
        display.print("Exhaust 1:");
        display.setCursor(70, 30);
        display.print(FVALUE3, DEC);
        display.setCursor(0, 40);
        display.print("Exhaust 2:");
        display.setCursor(70, 40);
        display.print(FVALUE4, DEC);
        display.display();

      }
*/
// Status Screen Start
    if (buttonPushCounter == 13) {
      if (CANfilterset < 3) {
        CAN0.init_Mask(0, 0, 0xFFFF);
        CAN0.init_Filt(0, 0, 0x00FA);

        CANfilterset++;

        resetMaxMin();
      }

      if (CAN_MSGAVAIL == CAN0.checkReceive())                        // Check if data coming
      {
        digitalWrite(led, HIGH);                                      // turn the LED on, Data RX

        unsigned int canId = "0x00FA";
        CAN0.readMsgBuf(canId, &rxLen, rxBuf);                               // Read data,  rxLen: data length, rxBuf: data buf

        if (rxBuf[0] == 13) {
          STATUS1 = word(rxBuf[6], rxBuf[7]);
          STATUS2 = bitRead(STATUS1, 15)*4 + bitRead(STATUS1, 14)*2 + bitRead(STATUS1, 13);   // Anti-Lag
          STATUS3 = bitRead(STATUS1, 12)*2 + bitRead(STATUS1, 11);                            // Launch Control
          STATUS4 = bitRead(STATUS1, 10)*4 + bitRead(STATUS1, 9)*2 + bitRead(STATUS1, 8);   // Traction Control
          STATUS5 = bitRead(STATUS1, 7)*8 + bitRead(STATUS1, 6)*4 + bitRead(STATUS1, 5)*2 + bitRead(STATUS1, 4);   // Cruise Control
        }
/*  Cutting due to space limit
          if (rxBuf[0] == 12) {
          CLIMIT1 = word(rxBuf[6], rxBuf[7]);
          if (bitRead(CLIMIT1, 0) == 1) (LIMIT1=0);}
          else if (bitRead(CLIMIT1, 1) == 1) (LIMIT1=1);
          else if (bitRead(CLIMIT1, 2) == 1) (LIMIT1=2);
          else if (bitRead(CLIMIT1, 3) == 1) (LIMIT1=3);
          else if (bitRead(CLIMIT1, 4) == 1) (LIMIT1=4);
          else if (bitRead(CLIMIT1, 5) == 1) (LIMIT1=5);
          else if (bitRead(CLIMIT1, 6) == 1) (LIMIT1=6);
          else if (bitRead(CLIMIT1, 7) == 1) (LIMIT1=7);
          else if (bitRead(CLIMIT1, 8) == 1) (LIMIT1=8);
          else if (bitRead(CLIMIT1, 9) == 1) (LIMIT1=9);
          else if (bitRead(CLIMIT1, 10) == 1) (LIMIT1=10);
          else if (bitRead(CLIMIT1, 11) == 1) (LIMIT1=11);
          else if (bitRead(CLIMIT1, 12) == 1) (LIMIT1=12);
          else if (bitRead(CLIMIT1, 13) == 1) (LIMIT1=13);
          else if (bitRead(CLIMIT1, 14) == 1) (LIMIT1=14);
          else if (bitRead(CLIMIT1, 15) == 1) (LIMIT1=15);          
*/          
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setFont(NULL);
    display.setCursor(0, 0);
    display.println("STATUS");
// Anti-Lag Status
        display.setCursor(0, 10);
        display.print("AntiLag:");
        display.setCursor(55, 10);
        if (STATUS2 == 0) drawRightString("OFF", 55, 10);
        else if (STATUS2 == 1 ) drawRightString("Armed", 55, 10);
        else if (STATUS2 == 2 ) drawRightString("OFF: RPM<500", 55, 10);
        else if (STATUS2 == 3 ) drawRightString("Armed:Cyc OFF", 55, 10);
        else if (STATUS2 == 4 ) drawRightString("Armed:Cyc ACT", 55, 10);
        else if (STATUS2 == 5 ) drawRightString("Cyc Cooldown", 55, 10);
        else if (STATUS2 == 6 ) drawRightString("D-Arm:Cyc ACT", 55, 10);

// Launch Control
        display.setTextSize(1);
        display.setCursor(0, 20);
        display.print("Launch: ");
        display.setTextSize(1);
        display.setCursor(55, 20);
        if (STATUS3 == 0) drawRightString("OFF", 55, 20);
        else if (STATUS3 == 1 ) drawRightString("Active", 55, 20);
        else if (STATUS3 == 2 ) drawRightString("Inactive", 55, 20);

// Traction Control
        display.setTextSize(1);
        display.setCursor(0, 30);
        display.print("Traction: ");
        display.setTextSize(1);
        display.setCursor(55, 30);
        if (STATUS4 == 0) drawRightString("OFF", 55, 30);
        else if (STATUS4 == 1 ) drawRightString("RPM Lockout", 55, 30);
        else if (STATUS4 == 2 ) drawRightString("TPS Lockout", 55, 30);
        else if (STATUS4 == 3 ) drawRightString("SPD Lockout", 55, 30);
        else if (STATUS4 == 4 ) drawRightString("Ready", 55, 30);
        else if (STATUS4 == 5 ) drawRightString("Active", 55, 30);
        else if (STATUS4 == 6 ) drawRightString("Disabled", 55, 30);
        else if (STATUS4 == 7 ) drawRightString("OFF: TQ Mod", 55, 30);

 // Cruise Control
        display.setTextSize(1);
        display.setCursor(0, 40);
        display.print("Cruise: ");
        display.setTextSize(1);
        display.setCursor(55, 40);
        if (STATUS5 == 0) drawRightString("OFF", 55, 40);
        else if (STATUS5 == 1 ) drawRightString("Enabled", 55, 40);
        else if (STATUS5 == 2 ) drawRightString("Active", 55, 40);
        else if (STATUS5 == 3 ) drawRightString("Startup LCK", 55, 40);
        else if (STATUS5 == 4 ) drawRightString("Min RPM", 55, 40);
        else if (STATUS5 == 5 ) drawRightString("Max RPM", 55, 40);
        else if (STATUS5 == 6 ) drawRightString("CAN Error", 55, 40);

// Limits
/* Cutting due to space limits
        display.setTextSize(1);
        display.setCursor(0, 50);
        display.print("Limit: ");
        display.setTextSize(1);
        display.setCursor(55, 50);
        if (LIMIT1 == 0) drawRightString("RPM Limit", 60, 55);
        else if (LIMIT1 == 1 ) drawRightString("MAP Limit", 60, 55);
        else if (LIMIT1 == 2 ) drawRightString("Speed Limit", 60, 55);
        else if (LIMIT1 == 3 ) drawRightString("MAX Ignition", 60, 55);
        else if (LIMIT1 == 4 ) drawRightString("ALag IGN Cut", 60, 55);
        else if (LIMIT1 == 5 ) drawRightString("High Voltage", 60, 55);
        else if (LIMIT1 == 6 ) drawRightString("Overrun Flag", 60, 55);
        else if (LIMIT1 == 7 ) drawRightString("Traction Lim", 60, 55);
        else if (LIMIT1 == 8 ) drawRightString("Low Voltage", 60, 55);
        else if (LIMIT1 == 9 ) drawRightString("Launch RPM L", 60, 55);
        else if (LIMIT1 == 10 ) drawRightString("Empty", 60, 55);
        else if (LIMIT1 == 11 ) drawRightString("GP RPM 1 Lim", 60, 55);
        else if (LIMIT1 == 12 ) drawRightString("Rotary Oil", 60, 55);
        else if (LIMIT1 == 13 ) drawRightString("GP RPM 2 Lim", 60, 55);
        else if (LIMIT1 == 14 ) drawRightString("EThrottle", 60, 55);
        else if (LIMIT1 == 15 ) drawRightString("Cyclic Idle", 60, 55);
*/
      }
    }
/*
 * Link Generic dash profile uses Frame 14 bytes 6-7 for status for Anti-lag, Launch Control, Traction Control and Cruise Control
 * In the Ardunio CAN, this translates to the first byte of the frame == 13 setting the fram number to 14. Bytes 6 and 7 and read into the buffer
 * the bit ordering on the Ardunio is as follows
 * 
 * |   AL   |  LC |  TC  |  CC   |  NA   |
 * |15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0|
 * 
 * AL = Anti-Lag
 * LC = Launch Control
 * TC = Traction Control
 * CC = Cruise Control
 * NA = Unsued
 */

        display.display();
  }
    if (buttonPushCounter == 14) {
      if ( ScreenOff < 500) {

        ScreenOff++;

        display.setTextSize(2);
        display.setCursor(5, 25);
        display.println("SCREEN OFF");
        display.display();

      }

      else {
        display.clearDisplay();
        display.display();
      }
      }
}
void resetValues() {
  //ECU VALUES
  VALUE1 = 0;
  VALUE2 = 0;
  FVALUE1 = 0;
  FVALUE2 = 0;
}
void resetMaxMin() {

  //SCREEN 1 HEADING, DATA AND MAX/MIN VALUES
  MAX1 = -10000;
  MAX2 = -10000;
  MIN1 = 10000;
  MIN2 = 10000;
  FMAX1 = -10000;
  FMAX2 = -10000;
  FMIN1 = 10000;
  FMIN2 = 10000;
}
