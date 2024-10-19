/******************************************************************************************
 * Project    : MY Jeep Compass Utility/MyGIG RHP
 * Hack for my Jeep Compass MyGIG RHP and Utility
 * Features:
 *  - Emulating VES presense to enable VIDEO AUX IN in MyGIG head unit
 *  - Enable intelligent cornering light
 *  - Enable digital output when pressing the steering wheel button
 *  - Enable digital output whrn pressing fobik Trunk button
 *
 * Copyright (C) 2024 DMITRY GOSUDAR <gosudar1@narod.ru>
 * http://gosudar.org.ru
 *
 * This is free software. You may use/redistribute it under The MIT License terms.
 *
 * 
 * 
 * Dependencies:
 * CAN Library by Sandeep Mistry V 0.3.1
 * https://github.com/sandeepmistry/arduino-CAN
 * 
 ******************************************************************************************/

#include <CAN.h>

volatile uint8_t timeH = 0, timeM = 0, timeS = 0;  //The radio does not keep time, it only sets and displays time.

uint8_t keyState = 0x00;                  //initial state = key-in, accessory on

String SerialRXBuffer = "";
bool SerialRXSpecial = false;
bool FrontFogON = false;// Включены ли туманки? true-on, false-off
bool EnableTempFog = 0;//Временная переменная для отслеживания статуса Enable Fog
bool RightFog = false;// Включать правую туманку? true-on, false-off
bool LeftFog = false;// Включать левую туманку? true-on, false-off
bool Steering_Wheel_1_flag = false;//флаг длительного нажатия true-on, false-off
bool RKE_Trunk_Button_flag = false;

int EnableFogLeft = 7; // Инициализация переменной EnableFogLeft к выводу 7
int EnableFogRight = 6;// Инициализация переменной EnableFogRight к выводу 6
int Steering_Wheel_1 = 4;// Инициализация переменной Steering_Wheel_1 к выводу 4
int RKE_Trunk_Button = 4;// Инициализация переменной Steering_Wheel_1 к выводу 3

int Temp_Button_SW1 = 0;//счетчик удержания левой центральной подрулевой кнопки


//Define subroutines
void Enable_VES();
void Check_FOG();
void Check_Steering_Wheel();
void Check_RKE_Button();

void setup()
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 62500;            // compare match register 16MHz/256
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();
  
  Serial.begin(115200);

  //these settings are dependant on the CAN module being used. CS could be pin 9 or 10
  //the clockk can be 8 or 16 MHz
  CAN.setPins(10, 2);
  //CAN.setPins(9, 2);
  CAN.setClockFrequency(8E6);
  //CAN.setClockFrequency(16E6);

  pinMode(EnableFogLeft, OUTPUT); // Установим вывод как выход
  digitalWrite(EnableFogLeft, LOW); // Устанавливаем на нем 0 (выкл)
  pinMode(EnableFogRight, OUTPUT); // Установим вывод как выход
  digitalWrite(EnableFogRight, LOW);// Устанавливаем на нем 0 (выкл)
  pinMode(Steering_Wheel_1, OUTPUT); // Установим вывод как выход
  digitalWrite(Steering_Wheel_1, LOW);// Устанавливаем на нем 0 (выкл)
  pinMode(RKE_Trunk_Button, OUTPUT); // Установим вывод как выход
  digitalWrite(RKE_Trunk_Button, LOW);// Устанавливаем на нем 0 (выкл)
 
  if (!CAN.begin(83E3))      //start the CAN bus at 83.333 kbps 
  {
    Serial.println("Starting CAN0 failed!");
    while (1);
  }

  CAN.onReceive(onCANReceive);

  Serial.println("MY Jeep Compass utility start:");

}

ISR(TIMER1_COMPA_vect)
{
  timeS++;
  if (timeS > 59)
  {
    timeS = 0;
    timeM++;
  }
  if (timeM > 59)
  {
    timeM = 0;
    timeH++;
  }
  if (timeH > 23)
    timeH = 0;
}

void loop()
{
  for (uint16_t y = 0; y < 900; y++)  //~900mS delay while checking serial.
  {
    delay(1);
    checkSerial();
  }

   //Enable_VES();  // Enable VES - moved to Check_Steering_Wheel
   Check_FOG();   // Check FOG 
   Check_Steering_Wheel();  // Check Steering Wheel buttons
   Check_RKE_Button();      // Check RKE fobic buttons
   //delay(30);
}

void Enable_VES()
{
  if ( keyState == 0x00 )
    delay(30);
  else
  {
    // VES Lockpic
    canSend(0x322, 0x01, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00); delay(25); //Ves configuration
    canSend(0x3B4, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x07); delay(25); //Ves AUX VIDEO
  }
}

void Check_RKE_Button()
{
  //RKE_Trunk_Button
  if (RKE_Trunk_Button_flag == true)
  {
    if (digitalRead(RKE_Trunk_Button) == 0)
    { 
      Serial.println("---Fobik Key Enabled = OUT ON ---");
      digitalWrite(RKE_Trunk_Button, HIGH);
      delay(5);
    }
    else
    {
      Serial.println("---Fobik Key Enabled = OUT OFF ---");
      digitalWrite(RKE_Trunk_Button, LOW);
      delay(5);
    }
    RKE_Trunk_Button_flag = false;
  }
}

void Check_Steering_Wheel()
{
  //Serial.println("---SW ---");
  if (Steering_Wheel_1_flag == true)
  {
    Temp_Button_SW1 = 0;//сброс счетчика нажатий

    Enable_VES();  // Enable VES
       
    if (digitalRead(Steering_Wheel_1) == 0)
    { 
      Serial.println("---Steering Wheel Key Enabled = OUT ON ---");
      digitalWrite(Steering_Wheel_1, HIGH);
      delay(5);
    }
    else
    {
      Serial.println("---Steering Wheel Key Enabled = OUT OFF ---");
      digitalWrite(Steering_Wheel_1, LOW);
      delay(5);
    }
   Steering_Wheel_1_flag = false;
  }
}

void Check_FOG()
{
  if (FrontFogON == false)
  {
    // Front Fog disable
    if ( RightFog == true )
    {
      //Fog right enable
      EnableTempFog = true;
      digitalWrite(EnableFogRight, HIGH);
      Serial.println("---Right Fog ON---"); 
    }
    else
    {
      //Fog right disable
      EnableTempFog = false;

      if (digitalRead(EnableFogRight) != 0)
      { 
        digitalWrite(EnableFogRight, LOW);
        delay(5);
      }  
      //Serial.println("---Right Fog OFF---");
    }

    if ( LeftFog == true )
    {
      //Fog left enable
      EnableTempFog = true;
      digitalWrite(EnableFogLeft, HIGH); 
      Serial.println("---Left Fog ON---"); 
    }
    else
    {
      //Fog left disable
      EnableTempFog = false;

      if (digitalRead(EnableFogLeft) != 0)
      { 
        digitalWrite(EnableFogLeft, LOW);
        delay(5);
      } 
      //Serial.println("---Left Fog OFF---"); 
    }   
  }
  else
  {
    // Front Fog enable
    if (EnableTempFog == true)
    {
      EnableTempFog = false;
      Serial.println("---ALL Fog OFF---");
      digitalWrite(EnableFogLeft, LOW);
      digitalWrite(EnableFogRight, LOW);
    }
  }   
}

void onCANReceive(int packetSize) 
{
  if (CAN.packetRtr()) 
  {
    Serial.print("RTR ID 0x");
    Serial.print(CAN.packetId(), HEX);
    Serial.print(" Requested Length ");
    Serial.println(CAN.packetDlc());
    return;
  }

  uint8_t parameters[8];
  uint32_t packetID = CAN.packetId();
  for (uint8_t x = 0; x < packetSize; x++)
    parameters[x] = CAN.read();
        
  switch (packetID)
  { 
    case 0x000:
      //keyState: 00 = no key, 01 = key in, 41 = accessory, 81 = run, 21 = start
      keyState = parameters[0];
      break;

    case 0x015:
      if ( parameters[4] == 0x12 || parameters[4] == 0x13 ) //Left
      //if ( parameters[4] == 0x13 ) //Left
      {
        LeftFog = true; // Enable Left Fog Lamp
      }
      else
      {
        LeftFog = false; // Disable Left Fog Lamp
      }

      if ( parameters[4] == 0x0C || parameters[4] == 0x0D ) //Right
      //if ( parameters[4] == 0x0C ) //Right
      {
        RightFog = true; // Enable Right Fog Lamp
      }
      else
      {
        RightFog = false; // Disable Right Fog Lamp
      }
      break;

    case 0x006:
      if ( parameters[0] == 0x48 || parameters[0] == 0x58 || parameters[0] == 0x78 || parameters[0] == 0xD8 || parameters[0] == 0xD9 || parameters[0] == 0xF8 || parameters[0] == 0xDA ) //Check front fog on
      {
        FrontFogON = true; // FrontFogON - Front fog ON - true
      }
      else
      {
        FrontFogON = false; // FrontFogON - Front fog OFF - false
      }
      break;

    case 0x3A0:
      if ( parameters[0] == 0x20 ) //левый подрулевой середина
      {
        // Нажата кнопка проверяем кол-во раз
        if ( Temp_Button_SW1 == 5 )
        {
           // длительное нажатие.
           Steering_Wheel_1_flag = true; // делаем что надо
           Serial.print("---Key Enabled----"); Serial.println();
        }
        else
        {
           Temp_Button_SW1 += 1; //увеличиваем счетчик нажатия
        }
      }
      else
      {
        Temp_Button_SW1 = 0;//сброс счетчика нажатий
      }
      break;

    case 0x012:
      if ( parameters[0] == 0x05 ) //RKE key Trunk
      {
        RKE_Trunk_Button_flag = true; // Enable RKE key Trunk
        Serial.print("---Fobic Key Enabled----"); Serial.println();
      }
    break;

    
    default:
      //Output information from unexpected packets
//      Serial.print("0x");
//      Serial.print(packetID, HEX);
//      Serial.print(" defaulted-ID size: ");
//      Serial.print(packetSize);
      for (uint8_t x = 0; x < packetSize; x++)
      {
//        Serial.print(" 0x"); Serial.print(parameters[x], HEX);
      }
//      Serial.println();
  }
}

void checkSerial()
{
  if (Serial.available())
  {
    char RX = Serial.read();
    if (!SerialRXSpecial)
    {
      if ( RX == 'I' || RX == 'i' ) //power on
      {
        keyState = 0x41;
        canSend(0x000, keyState, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);
      }
      if ( RX == 'O' || RX == 'o' ) //power off
      {
        keyState = 0x00;
        canSend(0x000, keyState, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);
      }

      if ( RX == 'T' || RX == 't' ) //set time
      {
        SerialRXBuffer = RX;
        SerialRXSpecial = true;
      }
    }
    else
    {
      SerialRXBuffer += RX;
      if (SerialRXBuffer.length() >= 5)
      {
        String tempVal = "";
        char tempArray[8];
        tempVal = SerialRXBuffer.substring(1,3);
        tempVal.toCharArray(tempArray,sizeof(tempArray));
        timeH = strtol(tempArray, 0, 0);
        tempVal = SerialRXBuffer.substring(3,5);
        tempVal.toCharArray(tempArray,sizeof(tempArray));
        timeM = strtol(tempArray, 0, 0);
        SerialRXBuffer = "";
        SerialRXSpecial = false;
      }
    }
  }
}


//helper functions...

void canSend(uint32_t ID, uint8_t b0)
{
  uint8_t b[1];
  b[0] = b0;
  canTX(1, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1)
{
  uint8_t b[2];
  b[0] = b0; b[1] = b1;
  canTX(2, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2)
{
  uint8_t b[3];
  b[0] = b0; b[1] = b1; b[2] = b2;
  canTX(3, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  uint8_t b[4];
  b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3;
  canTX(4, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
  uint8_t b[5];
  b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; b[4] = b4;
  canTX(5, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5)
{
  uint8_t b[6];
  b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; b[4] = b4; b[5] = b5;
  canTX(6, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6)
{
  uint8_t b[7];
  b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; b[4] = b4; b[5] = b5; b[6] = b6;
  canTX(7, ID, b);
}
void canSend(uint32_t ID, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  uint8_t b[8];
  b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; b[4] = b4; b[5] = b5; b[6] = b6; b[7] = b7; 
  canTX(8, ID, b);
}

void canTX(uint8_t packetSize, uint32_t ID, uint8_t b[])
{
  CAN.beginPacket(ID, packetSize);
  CAN.write(b, packetSize);
  CAN.endPacket();
}
