/******************************************************************************************
 * Project    : MY Jeep Compass Utility/MyGIG RHP
 * Hack for my Jeep Compass MyGIG RHP and Utility
 * Version 1.4
 * Features:
 *  - Emulating VES presense to enable VIDEO AUX IN in MyGIG head unit
 *  - Enable intelligent cornering light
 *  - Enable digital output when pressing the steering wheel button
 *  - Enable digital output when pressing fobik Trunk button
 *  - Enable digital output when pressing fobik Lock button
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

uint8_t keyState = 0x00;                  //initial state = key-in, accessory on
uint8_t Trunk_reset_counter_az = 0x00;       //reset az with fobik trunk button 00 - disable, 01 - enable

String SerialRXBuffer = "";
bool SerialRXSpecial = false;
bool Engine_Run = false;// Статус двигателя true - run, false - off
bool FrontFogON = false;// Включены ли туманки? true-on, false-off
bool EnableTempFog = 0;//Временная переменная для отслеживания статуса Enable Fog
bool RightFog = false;// Включать правую туманку? true-on, false-off
bool LeftFog = false;// Включать левую туманку? true-on, false-off
bool Steering_Wheel_1_flag = false;//флаг длительного нажатия true-on, false-off
bool RKE_Trunk_Button_flag = false;//флаг нажатия кнопки багажника true-on, false-off
bool RKE_Alarm_ON_flag = false;//флаг нажатия кнопки постановки на охрану true-on, false-off
bool Remote_start;//Remote start true-on, false-off
bool Jeep_Alarm_Status;//Статус охраны true-on, false-off

int EnableFogLeft = 7; // Инициализация переменной EnableFogLeft к выводу 7
int EnableFogRight = 6;// Инициализация переменной EnableFogRight к выводу 6
int RKE_Alarm_ON = 5;// Инициализация переменной Alarm_ON к выводу 5
int Steering_Wheel_1 = 4;// Инициализация переменной Steering_Wheel_1 к выводу 4
int RKE_Trunk_Button = 3;// Инициализация переменной Steering_Wheel_1 к выводу 3

int Temp_Button_SW1 = 0;//счетчик удержания левой центральной подрулевой кнопки
int reset_az_stage = 0;//счетчик кол-ва remote start

int Jeep_RPM = 0;// обороты двигателя
int Jeep_Speed = 0;// скорость авто
uint8_t Jeep_Gear = 0;// статус передачи вариатора (акпп)

float Jeep_Temp_Outdoor = 0;// температура за бортом
float Jeep_Batt;// бортовое напряжение

uint32_t my_reset_az;

//Define subroutines
//void Enable_VES();
void Check_FOG();
void Check_Steering_Wheel();
void Check_RKE_Button();
void Check_Counter_AZ();

void setup()
{ 
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
  pinMode(RKE_Alarm_ON, OUTPUT); // Установим вывод как выход
  digitalWrite(RKE_Alarm_ON, LOW);// Устанавливаем на нем 0 (выкл)
 
  if (!CAN.begin(83E3))      //start the CAN bus at 83.333 kbps 
  {
    Serial.println("Error! Starting CAN0 failed!");
    while (1);
  }

  CAN.onReceive(onCANReceive);
  Serial.println("MY Jeep Compass utility v 1.4 start:");
}

void loop()
{
  // checkSerial() for testing
  for (uint16_t y = 0; y < 900; y++)  //~900mS delay while checking serial.
  {
    delay(1);
    checkSerial();
  }

   //Enable_VES();          // Enable VES - moved to Check_Steering_Wheel
   Check_FOG();             // Check FOG 
   Check_Steering_Wheel();  // Check Steering Wheel buttons
   Check_RKE_Button();      // Check RKE fobic buttons
   Check_Counter_AZ();      // Check counter Remote start
   //delay(30);
}

void Enable_VES()
{
  if ( keyState == 0x00 )
    delay(30);
  else
  {
    // VES Lockpic for Mygig
    canSend(0x322, 0x01, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00); delay(25); //Ves configuration
    canSend(0x3B4, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x07); delay(25); //Ves AUX VIDEO
  }
}

void Check_RKE_Button()
{
  //RKE_Trunk_Button
  if (RKE_Trunk_Button_flag == true)
  {
    if ( Trunk_reset_counter_az == 0x01 )
    {
      reset_counter_az();// to do: reset counter remote start
    }
    if (digitalRead(RKE_Trunk_Button) == 0)
    { 
      Serial.println("---Fobik Key Trunk Enabled = OUT ON ---");
      digitalWrite(RKE_Trunk_Button, HIGH);
      delay(5);
    }
    else
    {
      Serial.println("---Fobik Key Trunk Enabled = OUT OFF ---");
      digitalWrite(RKE_Trunk_Button, LOW);
      delay(5);
    }
    RKE_Trunk_Button_flag = false;
  }
  //RKE_Alarm_ON
  if (RKE_Alarm_ON_flag == true)
  {
    // Fobik Key Enabled Lock
    if (digitalRead(RKE_Alarm_ON) == 0)
    { 
      Serial.println("---Fobik Key Lock Enabled = Lock Alarm ON ---");
      digitalWrite(RKE_Alarm_ON, HIGH);
      delay(500);
      digitalWrite(RKE_Alarm_ON, LOW);
    }
    RKE_Alarm_ON_flag = false;
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
      if (parameters[0] == 0x81)
      {
        Engine_Run = true;
      }
      else
      {
        Engine_Run = false;
      }    
      break;

    case 0x015:
      Jeep_Batt = parameters[1] / 10;
      Jeep_Temp_Outdoor = parameters[0] / 10;
      
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
      if (parameters[0] == 0x08 and (parameters[1] == 0x01 or parameters[1] == 0x02 or parameters[1] == 0x03 or parameters[1] == 0x04 or parameters[1] == 0x05))
      {
        reset_az_stage += 1;
        Serial.print("---Counter Remote Start Updated = ");Serial.print(reset_az_stage);Serial.print("----"); Serial.println();
        my_reset_az = millis();
      }
      if ( parameters[0] == 0x05 ) //RKE key Trunk
      {
        RKE_Trunk_Button_flag = true; // Enable RKE key Trunk
        Serial.print("---Fobic Key Enabled Trunc----"); Serial.println();
      }
      if ( parameters[0] == 0x01 or parameters[0] == 0x09 ) //RKE key Lock (Alarm ON)
      {
        RKE_Alarm_ON_flag = true; // Enable RKE key Lock (Alarm ON)
        Serial.print("---Fobic Key Enabled Lock Alarm ON----"); Serial.println();
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

void Check_Counter_AZ()
{
  if (keyState == 0x81 or keyState == 0x41 or keyState == 0x61)
  {
    //reset counter AZ stage=0
    Serial.print("---Key-in Stop RESET counter az--- ");Serial.println();
    reset_az_stage = 0;
  }
  
  if (reset_az_stage == 2)
  {
    if (millis() - my_reset_az >= 9999) 
    {
      // сброс счетчика количества АЗ
      Serial.print("---RESET COUNTER AZ START--- ");Serial.println();
      reset_counter_az();
      reset_az_stage = 0;
      my_reset_az = millis();
    }
  }
}

void reset_counter_az()
{
  // сброс счетчика количества АЗ
  // canId 11D, byte0=40 - Flash High Beams
  Serial.println("---Reset counter AZ----");
  canSend(0x11D, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00);// Flash High Beam. Мигнем фарами
  // to do function reset 
  delay(5);
}

void checkSerial()
{
  if (Serial.available())
  {
    char RX = Serial.read();
    if (!SerialRXSpecial)
    {
      if ( RX == 'R' || RX == 'r' ) //RKE
      {
        //keyState = 0x41;
        //RKE_Trunk_Button_flag = true;
        //canSend(0x000, keyState, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);
      }
      if ( RX == 'A' || RX == 'a' ) //AZ
      {
        //keyState = 0x00;
        //canSend(0x000, keyState, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);       
      }
      if ( RX == 'Z' || RX == 'z' ) //AZ
      {
        //keyState = 0x41;
        //Check_Counter_AZ();
        //keyState = 0x00;
        //canSend(0x000, keyState, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);       
      }
      if ( RX == 'T' || RX == 't' )
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
