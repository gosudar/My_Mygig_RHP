/******************************************************************************************
 * Project    : MY Jeep Compass Utility/MyGIG RHP
 * Hack for my Jeep Compass MyGIG RHP and Utility
 * * Version 2.2.1
 * Features:
 *  - Emulating VES presense to enable VIDEO AUX IN in MyGIG head unit
 *  - Enable intelligent cornering light
 *  - Enable digital output when pressing the steering wheel button
 *  - Enable digital output when pressing fobik Trunk button
 *  - Close mirrorws when 2x pressed fobik Lock button
 *  - Open mirrors when pressed fobik Unlock and engine run
 *  - Reset counter factory Remote Start (manual)
 *  - Activation hazards warning lights when reversing
 *  - Beeps with alarm on/alarm off
 *  - Auto auto-detection HSM (HeatSeatModule)
 *  - Enable heat seats with factory remote start
 *  - Added Demo Fog
 *  
 *  Settings:
 *  - Long press central left button - enable digital output for ip-tv
 *  - Long press central right button - enable/disable hasards with rear
 *  - Long press left up button - enable/disable beep with lock/unlock alarm
 *  - Long press left down button - enable open-close mirrors with temperature sensor
 *  - Long press right down button - enable demo fog
 *  
 *  Hardware - pins 10,2 ClockFrequency - 8E6
 *  
 *
 * Copyright (C) 2025 DMITRY GOSUDAR <gosudar1@narod.ru>
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

/****************************
 * Start Global settings special functions
 ****************************/
 bool Settings_VES = true;            // Разрешена эмуляция VES
 bool Settings_FOG = true;            // Разрешен подсвет поворота
 bool Settings_HOT_TEMP = false;      // Учитывать уличную температуру. Default - false
 bool Settings_HEAT_SEAT = false;     // Автоопределение HSM, включение обогрева при АЗ 
 /***************************
 * Stop Global special settings
 ****************************/
#include <CAN.h>

uint8_t keyState = 0x00;            // initial state = no-key, all-off

bool Engine_Run = false;            // Двигатель true - run, false - off
bool FrontFogON = false;            // Включены ли туманки? true-on, false-off
bool RightFog = false;              // Включать правую туманку? true-on, false-off
bool LeftFog = false;               // Включать левую туманку? true-on, false-off
bool Steering_Wheel_1_flag = false; // флаг длительного нажатия true-on, false-off
bool RKE_Trunk_Button_flag = false; // флаг нажатия кнопки "багажник" на фобике
bool RKE_Alarm_ON_flag = false;     // флаг постановки на охрану (для доводчика)
bool RKE_Alarm_OFF_flag = false;    // флаг снятия с охраны (для доводчика)
bool RKE_AZ_flag = false;           // флаг нажатия кнопки Remote Start
bool Remote_start = false;          // текущий статус Remote start true-on, false-off
bool Alarm_Status = false;          // текущий статус Alarm status true-on, false-off
bool CAN_LOGS = false;              // Логи кан шины в консоль true-on, false-off
bool BEEP = false;                  // статус бибип при постановке а охрану-снятия с охраны
bool Hasards_ON = false;            // включена аварийка с кнопки
bool Hasards_OFF = false;           // разрешение на включение аварийки при ЗХ

int Steering_Wheel_1 = 8;           // Инициализация переменной Steering_Wheel_1 к выводу 8
int EnableFogLeft = 7;              // Инициализация переменной EnableFogLeft к выводу 7
int EnableFogRight = 6;             // Инициализация переменной EnableFogRight к выводу 6
int Mirrors_Open = 5;               // Инициализация переменной Alarm_ON к выводу 5
int Mirrors_Close = 4;              // Инициализация переменной Alarm_OFF к выводу 4
int RKE_Trunk_Button = 3;           // Инициализация переменной RKE_Trunk_Button к выводу 3
int Temp_Button_SW1 = 0;            // счетчик удержания левой центральной подрулевой кнопки
int reset_az_stage = 0;             // счетчик кол-ва АЗ
//int Jeep_RPM = 0;                   // обороты двигателя

byte Jeep_Wiper = 0x01;             // текущий режим дворников
byte Jeep_Hasards = 0x02;           // текущий статус аварийки
byte Jeep_Speed = 0;                // скорость авто
byte Jeep_Gear = 0;                 // селектор коробки
byte Jeep_Temp_Outdoor = 0x51;      // температура за бортом
//float Jeep_Batt;                  // напряжение
byte Jeep_HeatSeat_Status_1 = 0;    // статус обогрева левого сидения
byte Jeep_HeatSeat_Status_2 = 0;    // статус обогрева правого сидения
byte Jeep_Defrost_Rear = 0;         // статус обогрева стекла
byte Demo_FOG = 0;                  // статус демо режима противотуманок
byte Mirrors_Open_Stage = 0;        // 01 - снято с охраны, 02 - двигатель работает
byte Mirrors_Close_Stage = 0;       // 01 - однократное нажатие, 02 - двойное нажатие

uint32_t my_reset_az;               // временная задержка для сброса счетчика АЗ
uint32_t my_hasard_on;              // временная задержка для аварийки при ЗХ
uint32_t my_mirrors;                // временная задержка для складывания зеркал

void setup()
{ 
  Serial.begin(115200);

  //these settings are dependant on the CAN module being used. CS could be pin 9 or 10
  //the clock can be 8 or 16 MHz
  CAN.setPins(10, 2);
  //CAN.setPins(9, 2);
  CAN.setClockFrequency(8E6);
  //CAN.setClockFrequency(16E6);

  pinMode(EnableFogLeft, OUTPUT);         // Установим вывод как выход
  digitalWrite(EnableFogLeft, HIGH);      // Устанавливаем на нем 1 (выкл)
  pinMode(EnableFogRight, OUTPUT);        // Установим вывод как выход
  digitalWrite(EnableFogRight, HIGH);     // Устанавливаем на нем 1 (выкл)
  pinMode(Steering_Wheel_1, OUTPUT);      // Установим вывод как выход
  digitalWrite(Steering_Wheel_1, HIGH);   // Устанавливаем на нем 1 (выкл)
  pinMode(RKE_Trunk_Button, OUTPUT);      // Установим вывод как выход
  digitalWrite(RKE_Trunk_Button, HIGH);   // Устанавливаем на нем 1 (выкл)
  pinMode(Mirrors_Open, OUTPUT);          // Установим вывод как выход
  digitalWrite(Mirrors_Open, HIGH);       // Устанавливаем на нем 1 (выкл)
  pinMode(Mirrors_Close, OUTPUT);         // Установим вывод как выход
  digitalWrite(Mirrors_Close, HIGH);      // Устанавливаем на нем 1 (выкл)
 
  if (!CAN.begin(83E3))      //start the CAN bus at 83.333 kbps 
  {
    Serial.println(".....Starting CAN 83.333kbps failed!.....");
    while (1);
  }

  CAN.onReceive(onCANReceive);
  Serial.println(".....  MY Jeep Compass utility start.");
}

void loop()
{
  for (uint16_t y = 0; y < 900; y++)  //~900mS delay while checking serial.
  {
    delay(1);
    checkSerial();
  }
 
  Enable_VES();           // Enable VES
  Check_FOG();            // Check FOG
  Check_Steering_Wheel(); // Check Steering Wheel buttons
  Check_RKE_Button();     // Check RKE fobic buttons
  Check_Counter_AZ();     // Check counter AZ
  Check_Hasards();        // Check Hasards ON
  Check_HeatSeat();       // Check Heat Seat
  demo_fog();             // Demo Fog
  Check_Mirrors();        // Check Mirrors
  delay(30);  
}

void Enable_VES()
{
  if (Settings_VES == true)
  {
    if ( keyState == 0x00)
      delay(1);
    else
    {
      // VES Lockpic
      canSend(0x322, 0x01, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00); delay(25); //Ves configuration
      canSend(0x3B4, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x07); delay(25); //Ves AUX VIDEO
    }
  }
}

void Check_Hasards()
{
  if ((Hasards_OFF == true) and (Jeep_Gear == 0x52) and (Engine_Run == true) and Jeep_Hasards == 0x00) // Задний ход, двигатель работает и ни аварийка, ни поворотники не мигают
  {
    if (Hasards_ON == true)
    {
      if (millis() - my_hasard_on > 500) 
      {	
        // Мигаем аварийкой	  		  
        canSend(0x11D, 0x01, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
        delay(10);
        canSend(0x11D, 0x01, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
        delay(10);
        canSend(0x11D, 0x01, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
        delay(10);
        canSend(0x11D, 0x01, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
        delay(10);       
        canSend(0x11D, 0x01, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
        delay(10);  
        Serial.print(F("---HASARDS ON----")); Serial.println();
        my_hasard_on = millis();
      }
      Serial.print(F("+++PAUSE HASARDS ON++++")); Serial.println();
    }
    else
    {
      if (millis() - my_hasard_on > 1000) 
      {
        my_hasard_on = millis();
        Hasards_ON = true;
      }
      Serial.print(F("+++WAIT HASARDS ON++++")); Serial.println();
    }
  }
  else
  {
    Hasards_ON = false;
    my_hasard_on = millis();
  }
}

void Check_Mirrors()
{
	if ((Settings_HOT_TEMP == true) and (Jeep_Temp_Outdoor > 0x50))
	{
    // Mirror Open
    if ( Mirrors_Open_Stage != 0)
    {
      if ( Mirrors_Open_Stage == 1)
      {
        // Оk. Снято с охраны, проверяем двигатель
        if (keyState == 0x81)
        {
          // двигатель работает, переходим к расскладыванию
          Serial.println(F("---Unlock, engine start ! ---"));
          Mirrors_Open_Stage = 2;
        }
      }
      if ( Mirrors_Open_Stage == 2)
      {
        //Охрана выкл, Двигатель работает, раскладываем зеркала   
        Serial.println(F("---Open mirrors ! ---"));     
        if (digitalRead(Mirrors_Open) != 0)
        {
          digitalWrite(Mirrors_Open, LOW);
          delay(500);
          digitalWrite(Mirrors_Open, HIGH);
        }
        else
        {
          Serial.println(F("---Mirrors Open = Ops!---"));
        }
        Mirrors_Open_Stage = 0;
      }
    }
	
    // Mirrors Close
    if (Mirrors_Close_Stage != 0)
    {
      if ( Mirrors_Close_Stage == 1)
      {
        // На охране
        // ждем второго нажатия кнопки
        if (millis() - my_mirrors >= 10000)
        {
          // время ожидания вышло. Сброс.
          Mirrors_Close_Stage = 0; 
          my_mirrors = millis();
          Serial.println(F("---Mirrors Close Time Reset = ---"));
        }
        else
        {
          // Wait
          Serial.println(F("---Mirrors Close Pause: Wait = ---"));
        }
      }
      if ( Mirrors_Close_Stage == 2)
      {
        //Двойное нажатие Lock   
        Serial.println(F("---Close mirrors !!!---"));     
        if (digitalRead(Mirrors_Close) != 0)
        {
          digitalWrite(Mirrors_Close, LOW);
          delay(500);
          digitalWrite(Mirrors_Close, HIGH);
        }
        else
        {
          Serial.println(F("---Mirrors Close = Ops!---"));
        }
        Mirrors_Close_Stage = 0;
      }
    }
  }
}

void Check_RKE_Button()
{
  //RKE_Trunk_Button
  if (RKE_Trunk_Button_flag == true)
  {
    // Fobik Trunk Key Enabled	
    if (digitalRead(RKE_Trunk_Button) == 0)
    { 
      Serial.println(F("---Fobik Key Enabled = Trunk OUT ON ---"));
      digitalWrite(RKE_Trunk_Button, HIGH);
      delay(5);
    }
    else
    {
      Serial.println(F("---Fobik Key Enabled = Trunk OUT OFF ---"));
      digitalWrite(RKE_Trunk_Button, LOW);
      delay(5);
    }
    RKE_Trunk_Button_flag = false;
  }
  
  //RKE_Alarm_ON
  if (RKE_Alarm_ON_flag == true)
  {
    if (BEEP == true)
    { 
      beep();// Beep
    }
    Serial.println(F("---Fobik Key Enabled = Alarm ON ---"));
	  
    Mirrors_Open_Stage = 0;// зеркала открывать не нужно
    Mirrors_Close_Stage += 1;// зеркала складывать нужно, след шаг
    if (Mirrors_Close_Stage == 1)
    { 
      my_mirrors = millis();// временная задержка 10сек 
    }	  
    RKE_Alarm_ON_flag = false;
  }

  //RKE_Alarm_OFF
  if (RKE_Alarm_OFF_flag == true)
  {
    // Fobik Key Enabled Alarm OFF    
    Serial.println(F("---Fobik Key Enabled = Alarm OFF ---"));
    Mirrors_Close_Stage = 0;
    Mirrors_Open_Stage = 1;
    if (BEEP == true)
    {
      beep();// Beep
      delay(100);
      beep();// Beep
    }
    RKE_Alarm_OFF_flag = false;
  }
}

void Check_Steering_Wheel()
{
  if (Steering_Wheel_1_flag == true)
  {
    if (Temp_Button_SW1 == 0x01)
    {
      if (Hasards_OFF == true)
      {
        Hasards_OFF = false;
        Serial.println(F("---Hasards OFF ---"));
        beep();// Beep
        delay(100);
        beep();// Beep
      }
      else
      {
        Hasards_OFF = true;
        Serial.println(F("---Hasards ON---"));
        // Beep
        beep();
      }
    }

    if (Temp_Button_SW1 == 0x20)
    { 
      if (digitalRead(Steering_Wheel_1) != 0)
      { 
        Serial.println(F("---Steering Wheel Key Enabled = OUT ON ---"));
        digitalWrite(Steering_Wheel_1, LOW);
        delay(5);
      }
      else
      {
        Serial.println(F("---Steering Wheel Key Enabled = OUT OFF ---"));
        digitalWrite(Steering_Wheel_1, HIGH);
        delay(5);
      }
      beep();// Beep
    }

    if (Temp_Button_SW1 == 0x08)
    {
      if (BEEP == true)
      {
        BEEP = false;
        Serial.println(F("---BEEP OFF ---"));
        beep();// Beep
        delay(100);
        beep();// Beep
      }
      else
      {
        BEEP = true;
        Serial.println(F("---BEEP ON---"));
        beep();// Beep
      }
    }
	
    if (Temp_Button_SW1 == 0x04)
    {
      if (Demo_FOG == 0)
      {
        Demo_FOG = 1;
        Serial.println(F("---Demo Fog Enabled ---"));
        beep();// Beep
      }
      else
      {
        Demo_FOG = 0;
        Serial.println(F("---Demo Fog Disabled ---"));
        beep();// Beep
        delay(100);
        beep();// Beep
      }
    }

    if (Temp_Button_SW1 == 0x10)
    {
      if (Settings_HOT_TEMP == true)
      {
        Settings_HOT_TEMP = false;
        Serial.println(F("---Use Temperature sensor Disable ---"));
        beep();// Beep
        delay(100);
        beep();// Beep
      }
      else
      {
        Settings_HOT_TEMP = true;
        Serial.println(F("---Use Temperature sensor Enable---"));
        beep();// Beep
      }
    }
    
    Temp_Button_SW1 = 0;//сброс счетчика нажатий
    Steering_Wheel_1_flag = false;
  }
}

void Check_FOG()
{ 
  if (Settings_FOG == true)
  {
    if ( (Engine_Run == true) && (FrontFogON == false) && (Jeep_Speed <= 9)  && (Jeep_Speed != 0) && (Jeep_Gear == 0x44))
    {
      // Front Fog disable
      if ( RightFog == true )
      {
        //Fog right enable
        if (digitalRead(EnableFogRight) != 0)
        { 
          digitalWrite(EnableFogRight, LOW);
          delay(5);
          Serial.println(F("---Right Fog ON---"));
        } 
      }
      else
      {
        //Fog right disable
        if (digitalRead(EnableFogRight) == 0)
        { 
          digitalWrite(EnableFogRight, HIGH);
          delay(5);
          Serial.println(F("---Right Fog OFF---"));
        }  
      }

      if ( LeftFog == true )
      {
        //Fog left enable
        if (digitalRead(EnableFogLeft) != 0)
        { 
          digitalWrite(EnableFogLeft, LOW);
          delay(5);
          Serial.println(F("---Left Fog ON---"));
        }   
      }
      else
      {
        //Fog left disable
        if (digitalRead(EnableFogLeft) == 0)
        { 
          digitalWrite(EnableFogLeft, HIGH);
          delay(5);
          Serial.println(F("---Left Fog OFF---")); 
        } 
      }   
    }
    else
    {
      // Front Fog enable. Left and right fogs disable
      if (digitalRead(EnableFogLeft) == 0)
      { 
        digitalWrite(EnableFogLeft, HIGH);
        delay(5);
        Serial.println(F("---ALL Fog OFF: Left---")); 
      }
      if (digitalRead(EnableFogRight) == 0)
      { 
        digitalWrite(EnableFogRight, HIGH);
        delay(5);
        Serial.println(F("---ALL Fog OFF: Right---"));
      } 
    }   
  }
}

void Check_HeatSeat()
{
  if (Settings_HEAT_SEAT == true)
  {
    // В глобальных настройках разрешено включать подогрев сидений при штатном АЗ
    if ((Remote_start == true) and (Engine_Run == true) and (Jeep_Defrost_Rear == 0x80))
    {
      // Двигатель работает на штатном АЗ и обогрев заднего стекла включен
      if (Jeep_Temp_Outdoor <= 0x50)
      {
        // Температура за бортом ниже нуля и в глобальных настройках разрешено по температуре
        if (Jeep_HeatSeat_Status_1 == 0x00)
        {
          // Подогрев левого сидения выключен
          canSend(0x02E, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00); delay(25); // включаем левый попогрей
        }
        if (Jeep_HeatSeat_Status_2 == 0x00)
        {
          // Подогрев правого сидения выключен
          canSend(0x02E, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00); delay(25); // включаем правый попогрей
        }
        Serial.println(F("---Heat Seat send to enable---"));
      }
      else
      {
        Serial.println(F("---Ops! NOT Low temp outdoor---"));
      }
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
      //keyState: 00 = no key, 01 = key in, 41 = accessory, 81 = run, 21 = start, 90 = Remote start
      keyState = parameters[0];
      if (parameters[0] == 0x81)
      {
        Engine_Run = true;
      }
      else
      {
        Engine_Run = false;
      }
      if (parameters[0] == 0x90)
      {
        Remote_start = true;
        Engine_Run = true;
      }
      else
      {
        Remote_start = false;
      }
	  
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Keystate: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
	  
    case 0x002:
      Jeep_Speed = parameters[2];
      break;

    case 0x003:
      Jeep_Gear = parameters[4];
      break;
	
    case 0x015:
      //Jeep_Batt = parameters[1] / 10.0;
      Jeep_Temp_Outdoor = parameters[0];
      
      if ( parameters[4] == 0x12 || parameters[4] == 0x13 ) //Left
      //if ( parameters[4] == 0x13 ) //Left
      {
        LeftFog = true; // Enable Left Fog Lamp
      }
      else
      {
        LeftFog = false; // Disable Left Fog Lamp
      }

      if ( parameters[4] == 0x0C || parameters[4] == 0x0D || parameters[4] == 0x0B ) //Right
      //if ( parameters[4] == 0x0C || parameters[4] == 0x0B ) //Right
      {
        RightFog = true; // Enable Right Fog Lamp
      }
      else
      {
        RightFog = false; // Disable Right Fog Lamp
      }
	  
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Temp,Batt other: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x006:
      if ( parameters[0] == 0x48 || parameters[0] == 0x58 || parameters[0] == 0x78 || parameters[0] == 0xD8 || parameters[0] == 0xD9 || parameters[0] == 0xF8 || parameters[0] == 0xDA ) //Check front fog on
      {
        FrontFogON = true; // FrontFogON - Front fog ON - true
        // Front Fog enable. Left and right fogs disable
        if (Settings_FOG == true)
        {
          kill_all_fog();
        }		
      }
      else
      {
        if ( parameters[0] == 0x18 || parameters[0] == 0x38 || parameters[0] == 0x19 || parameters[0] == 0x1A ) //Check front fog on
        {
          FrontFogON = false; // FrontFogON - Front fog OFF - false
        }
        else
        {
          if (Demo_FOG != 0)
          {
            //Serial.println(F("---FrontFogON = false---"));
            FrontFogON = false; // enabled Demo Fog
          }
          else
          {
            //Serial.println(F("---FrontFogON = true---"));
            FrontFogON = true; // only headlights
          }
        }
      }
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Headlights: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
    
    case 0x09C:
      Jeep_HeatSeat_Status_1 = parameters[0];
      Jeep_HeatSeat_Status_2 = parameters[1];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Heat Seat status: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
	
    case 0x0EC:
      Jeep_Defrost_Rear = parameters[0];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" HVAC - AC, Defrost: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
	
    case 0x11D:
      Jeep_Wiper = parameters[1];
      Jeep_Hasards = parameters[0];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" TIPM status - wiper, hasards: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x3A0:
      if ( parameters[0] == 0x20 or parameters[0] == 0x01 or parameters[0] == 0x08 or parameters[0] == 0x10 or parameters[0] == 0x04 ) //левый или правый подрулевой середина, левый подрулевой вверх или вниз
      {
        // Нажата кнопка проверяем кол-во раз
        if ( Temp_Button_SW1 == 5 )
        {
          // длительное нажатие.
          Steering_Wheel_1_flag = true; // делаем что надо
          Temp_Button_SW1 = parameters[0];
          Serial.print(F("---Steering Wheel Key Enabled----")); Serial.println();
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
	  
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Keys: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x012:      
      if (parameters[0] == 0x08 and (parameters[1] == 0x01 or parameters[1] == 0x02 or parameters[1] == 0x03 or parameters[1] == 0x04 or parameters[1] == 0x05 or parameters[1] == 0x06  or parameters[1] == 0x07  or parameters[1] == 0x08))
      {
        reset_az_stage += 1;
        Serial.print(F("---Counter Remote Start Updated = "));Serial.print(reset_az_stage);Serial.print(F("----")); Serial.println();
        my_reset_az = millis();
      }
      if ( parameters[0] == 0x05 ) //RKE key Trunk
      {
        RKE_Trunk_Button_flag = true; // Enable RKE key Trunk
        Serial.print(F("---Fobic Key Enabled Trunc----")); Serial.println();
      }
      if ( (parameters[0] == 0x01) and (parameters[1] == 0x01 or parameters[1] == 0x02 or parameters[1] == 0x03 or parameters[1] == 0x04 or parameters[1] == 0x05 or parameters[1] == 0x06  or parameters[1] == 0x07  or parameters[1] == 0x08) ) //RKE key Alarm ON
      {
        RKE_Alarm_ON_flag = true; // Enable RKE key Alarm ON
        Serial.print(F("---Fobic Key Enabled Alarm ON----")); Serial.println();
      }
      if (parameters[0] == 0x03 and (parameters[1] == 0x01 or parameters[1] == 0x02 or parameters[1] == 0x03 or parameters[1] == 0x04 or parameters[1] == 0x05 or parameters[1] == 0x06  or parameters[1] == 0x07  or parameters[1] == 0x08)) //RKE key Alarm OFF
      {
        RKE_Alarm_OFF_flag = true; // Enable RKE key Alarm OFF
        Serial.print(F("---Fobic Key Enabled Alarm OFF----")); Serial.println();
      }
      if ( (parameters[0] == 0x09) and (parameters[1] == 0x01 or parameters[1] == 0x02 or parameters[1] == 0x03 or parameters[1] == 0x04 or parameters[1] == 0x05 or parameters[1] == 0x06  or parameters[1] == 0x07  or parameters[1] == 0x08) ) //RKE key Alarm ON
      {
        RKE_AZ_flag = true; // Enable RKE key Remote Start
        Serial.print(F("---Fobic Key Enabled Remote Start----")); Serial.println();
      }
	  
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" RKE, IMMO, SKREEM: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x013:            
      if ( parameters[1] == 0x00 ) //Alarm_Status
      {
        Alarm_Status = false; // Alarm_Status OFF
        //Serial.print(F("---Alarm_Status OFF----")); Serial.println();
      }
      if ( parameters[1] == 0x01 or parameters[1] == 0x02 ) //Alarm_Status or Remote status
      {
        Alarm_Status = true; // Alarm_Status ON
        //Serial.print(F("---Alarm_Status ON----")); Serial.println();
      }
      if ( parameters[1] == 0x01) //Alarm_Status
      {
        if (digitalRead(Steering_Wheel_1) == 0)
        { 
          Serial.println(F("---Alarm ON: Disable Units Steering Wheel ---"));
          digitalWrite(Steering_Wheel_1, HIGH);
          delay(5);
        }
      }
	  
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Status alarm, doors, other: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x41B:
      Settings_HEAT_SEAT = true;// HSM installed
      break;
    
    default:
      if (CAN_LOGS == true)
      {
        //Output information from unexpected packets
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" defaulted-ID size: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
    //
  }
}

void Check_Counter_AZ()
{
  if (keyState == 0x81 or keyState == 0x41 or keyState == 0x61)
  {
    //reset counter AZ stage=0
    //Serial.print(F("---Key-in Stop RESET counter az--- "));Serial.println();
    reset_az_stage = 0;
  }
  
  if (reset_az_stage >= 2)
  {
    if (millis() - my_reset_az >= 9999) 
    {
      // сброс счетчика количества АЗ
      Serial.print(F("---RESET COUNTER AZ START--- "));Serial.println();
      reset_counter_az();
      reset_az_stage = 0;
      my_reset_az = millis();
    }
  }
}

void reset_counter_az()
{
  // сброс счетчика количества АЗ
  Serial.println(F("---Reset counter AZ----"));
  canSend(0x11D, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00);// Flash High Beam 
  Serial.println(F("---Stop Reset counter AZ----"));
}

void beep()
{
  // Beep
  // to do: check parameter Jeep_Hasards
  canSend(0x11D, 0x80, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
}

void demo_fog()
{
  if (Demo_FOG != 0)
  {
    if (Engine_Run == false)
    {
      kill_all_fog();
      Demo_FOG = 0;
    }
    //Demo_FOG
    if (FrontFogON == false)
    {
      //
      if ((Demo_FOG > 1) and (Demo_FOG <= 5))
      {
        //step 1 Right fog
        if (digitalRead(EnableFogLeft) == 0)
        { 
          digitalWrite(EnableFogLeft, HIGH);
          delay(5);
          Serial.println(F("---Demo Left Fog OFF---")); 
        }
        if (digitalRead(EnableFogRight) != 0)
        { 
          digitalWrite(EnableFogRight, LOW);
          delay(5);
          Serial.println(F("---Demo Right Fog ON---"));
        } 
      }
      
      if ((Demo_FOG > 6) and (Demo_FOG <= 10))
      {
        //step 2 Left Fog
        if (digitalRead(EnableFogRight) == 0)
        { 
          digitalWrite(EnableFogRight, HIGH);
          delay(5);
          Serial.println(F("---Demo Right Fog OFF---"));  
        }
        if (digitalRead(EnableFogLeft) != 0)
        { 
          digitalWrite(EnableFogLeft, LOW);
          delay(5);
          Serial.println(F("---Demo Left Fog ON---"));
        }
      }
	  
      Demo_FOG += 1;
	  
      if (Demo_FOG > 11)
      {
        //next step Demo FOG
        Demo_FOG = 1;
        if (digitalRead(EnableFogLeft) == 0)
        { 
          digitalWrite(EnableFogLeft, HIGH);
          delay(5);
          Serial.println(F("---Demo all Fog OFF: Left---")); 
        }
        if (digitalRead(EnableFogRight) == 0)
        { 
          digitalWrite(EnableFogRight, HIGH);
          delay(5);
          Serial.println(F("---Demo all Fog OFF: Right---"));
        } 
      }
      if (Engine_Run == false)
      {
        kill_all_fog();
        Demo_FOG = 0;
      }
    }
    else
    {
      kill_all_fog();
      Demo_FOG = 0;  
    }
  }
  else
  {
    kill_all_fog();
  }
}

void kill_all_fog()
{
  // Disable left and right fog
  if (digitalRead(EnableFogLeft) == 0)
  { 
    digitalWrite(EnableFogLeft, HIGH);
    delay(5);
    Serial.println(F("---Kill All Fog: Left---")); 
  }
  if (digitalRead(EnableFogRight) == 0)
  { 
    digitalWrite(EnableFogRight, HIGH);
    delay(5);
    Serial.println(F("---Kill All Fog: Right---"));
  } 
}

void checkSerial()
{
  if (Serial.available())
  {
    char RX = Serial.read();
    if ( RX == 'q' || RX == 'Q' ) //can logs on
    {
      CAN_LOGS = true;
      Serial.print(F("---CAN LOGS ON--- "));Serial.println();
    }
    if ( RX == 'w' || RX == 'W' ) //can logs off
    {
      CAN_LOGS = false;
      Serial.print(F("---CAN LOGS OFF--- "));Serial.println();
    }
    if ( RX == 'R' || RX == 'r' ) //RKE
    {
      reset_counter_az();
    }
    if ( RX == 'A' || RX == 'a' ) //
    {
      // wake on can 
      canSend(0x370, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00); delay(5);        
    }
    if ( RX == 'u' || RX == 'U' ) //
    {
       // wake on can 
       canSend(0x7FF, 0x00); delay(5);        
    }
    if ( RX == 'd' || RX == 'D' ) //
    {
      // demo fog
      Demo_FOG = 1;
      Serial.println(F("---Demo Fog Enabled ---"));        
    }
    if ( RX == 'b' ) //
    {
       // beep
       beep();        
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
