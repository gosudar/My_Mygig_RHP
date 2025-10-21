/******************************************************************************************
 * Project    : MY Jeep Compass Utility/MyGIG RHP
 * Hack for my Jeep Compass MyGIG RHP and Utility
 * * Version 2.4.2
 * Features:
 *  - Emulating VES presense to enable VIDEO AUX IN in MyGIG head unit
 *  - Auto-detection MyGIG and VES
 *  - Enable intelligent cornering light
 *  - Enable digital output when pressing the steering wheel button
 *  - Enable digital output when pressing fobik Trunk button
 *  - Close mirrors when 2x pressed fobik Lock button
 *  - Open mirrors when pressed fobik Unlock and engine run
 *  - Reset counter factory Remote Start (manual)
 *  - Activation hazards warning lights when reversing
 *  - Beeps with alarm on/alarm off
 *  - Auto-detection HSM (HeatSeatModule)
 *  - Auto-detection Rain Sensor (LRSM)
 *  - Enable heat seats with factory remote start
 *  - Added Demo Fog
 *  - Auto rear wiper (rain sensor)
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
 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 //! Управление релейной сборкой
 //! если по минусу, то раскомментировать следующие две строки
 #define HIGH 0x00
 #define LOW  0x01
 //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/****************************
 * Start Global settings special functions
 ****************************/
 bool Settings_VES = true;            // Эмуляция VES. Автоопределение VES на кан-шине.
 bool Settings_FOG = true;            // Разрешен подсвет поворота. 
 bool Settings_HOT_TEMP = false;      // Учитывать уличную температуру. Default - false.
 bool Settings_HEAT_SEAT = false;     // Автоопределение HSM, включение обогрева при АЗ. 
 bool Settings_Rain_Sensor = false;   // Автоопределение датчика дождя
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
bool Alarm_ON = false;              // постановка на охрану 
bool RKE_Alarm_OFF_flag = false;    // флаг снятия с охраны (для доводчика)
bool Alarm_OFF = false;             // снятие с охраны
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
int Mirrors_Open = 5;               // Инициализация переменной Mirrors_Open к выводу 5
int Mirrors_Close = 4;              // Инициализация переменной Mirrors_Close к выводу 4
int RKE_Trunk_Button = 3;           // Инициализация переменной RKE_Trunk_Button к выводу 3
int Temp_Button_SW1 = 0;            // счетчик удержания левой центральной подрулевой кнопки
int reset_az_stage = 0;             // счетчик кол-ва АЗ
int Jeep_RPM = 0;                   // обороты двигателя
//int Jeep_RKE_Code = 0;              // RKE key code
byte Jeep_Wiper = 0x01;             // текущий режим дворников
byte Jeep_Hasards = 0x02;           // текущий статус аварийки
byte Jeep_Speed = 0;                // скорость авто
byte Jeep_Gear = 0;                 // селектор коробки
byte Jeep_Temp_Outdoor = 0x51;      // температура за бортом
byte Jeep_Rain_Sensor = 0;          // датчик дождя
byte Jeep_HeatSeat_Status_1 = 0;    // статус обогрева левого сидения
byte Jeep_HeatSeat_Status_2 = 0;    // статус обогрева правого сидения
byte Jeep_Defrost_Rear = 0;         // статус обогрева стекла
byte Jeep_Demo_FOG = 0;             // статус демо режима противотуманок
byte Jeep_Mirrors_Open = 0;         // расскладывание зеркал
byte Jeep_Mirrors_Close = 0;        // складывание зеркал
float Jeep_Batt;                    // напряжение
uint32_t my_reset_az;               // временная задержка для сброса счетчика АЗ
uint32_t my_hasard_on;              // временная задержка для аварийки при ЗХ
uint32_t my_mirrors;                // временная задержка для складывания зеркал
uint32_t my_wiper_mirrors;          // временная задержка для заднего дворника

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
  digitalWrite(EnableFogLeft, LOW);       // Устанавливаем на нем 1 (выкл)
  pinMode(EnableFogRight, OUTPUT);        // Установим вывод как выход
  digitalWrite(EnableFogRight, LOW);      // Устанавливаем на нем 1 (выкл)
  pinMode(Steering_Wheel_1, OUTPUT);      // Установим вывод как выход
  digitalWrite(Steering_Wheel_1, LOW);    // Устанавливаем на нем 1 (выкл)
  pinMode(RKE_Trunk_Button, OUTPUT);      // Установим вывод как выход
  digitalWrite(RKE_Trunk_Button, LOW);    // Устанавливаем на нем 1 (выкл)
  pinMode(Mirrors_Open, OUTPUT);          // Установим вывод как выход
  digitalWrite(Mirrors_Open, LOW);        // Устанавливаем на нем 1 (выкл)
  pinMode(Mirrors_Close, OUTPUT);         // Установим вывод как выход
  digitalWrite(Mirrors_Close, LOW);       // Устанавливаем на нем 1 (выкл)
 
  if (!CAN.begin(83E3))      //start the CAN bus at 83.333 kbps 
  {
    Serial.println(".....Starting CAN 83.333kbps failed!.....");
    while (1);
  }

  CAN.onReceive(onCANReceive);
  Serial.println(".....  MY Jeep Compass utility start. version 2.4.2");
}

void loop()
{
  for (uint16_t y = 0; y < 900; y++)  //~900mS delay while checking serial.
  {
    delay(1);
    checkSerial();
  }
 
  Enable_VES();           // Проверяем VES
  Check_FOG();            // Проверяем туманки
  Check_Steering_Wheel(); // Проверяем рулевые кнопки
  Check_RKE_Button();     // Проверяем радио-кнопки фобика
  Check_Counter_AZ();     // Проверка штатного автозапуска
  Check_Hasards();        // Проверяем аварийку
  Check_HeatSeat();       // Проверяем обогрева сидений
  Check_Mirrors();        // Проверяем боковые зеркала
  Check_Alarm();          // Проверяем постановку, снятие с охраны 
  Check_Rain();           // Проверяем датчик дождя  
}

void Enable_VES()
{
  if (Settings_VES == true)
  {
    if ( keyState != 0x00)
    {
      // VES Lockpic
      canSend(0x322, 0x01, 0x70, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00); //Ves configuration
      canSend(0x3B4, 0x00, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x07); //Ves AUX VIDEO
    }
  }
}

void Check_Hasards()
{
  // Проверяем аварийку
  if ((Hasards_OFF == true) and (Jeep_Gear == 0x52) and (Engine_Run == true) and Jeep_Hasards == 0x00)
  {
    // Задний ход, двигатель работает и ни аварийка, ни поворотники не мигают
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
      // временная задержка миганий
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
  // Проверка боковых зеркал
  if ((Settings_HOT_TEMP == true) and (Jeep_Temp_Outdoor > 0x50)) // по температуре
  {
    // Открывание зеркал
    if (Jeep_Mirrors_Open != 0)
    { 
      // на всяк случай проверяем двойное нажатие + еще одно
      if (Jeep_Mirrors_Open == 3)
      {
        Jeep_Mirrors_Open = 2;
      }
      // и еще на всяк случай
      if (Jeep_Mirrors_Open == 6)
      {
        Jeep_Mirrors_Open = 5;
      }
      // и напоследок, еще на всяк случай
      if (Jeep_Mirrors_Open == 8)
      {
        Jeep_Mirrors_Open = 7;
      }

      if (Jeep_Mirrors_Open == 1)// снято с охраны
      {
        // Оk. Снято с охраны, проверяем двигатель
        if (keyState == 0x81 and Jeep_RPM > 500)
        {
          //снято с охраны и двигатель работает, переходим к расскладыванию
          Serial.println(F("---Unlock and engine start ! ---"));
          Jeep_Mirrors_Open = 5;
        }
      }
      if (Jeep_Mirrors_Open == 2)// двойное нажатие открыть
      {
        //двойное нажатие, переходим к расскладыванию
        Serial.println(F("---Unlock 2x. Open mirrors ! ---"));
        Jeep_Mirrors_Open = 5;
      }
      if (Jeep_Mirrors_Open == 5)// 05 - расскладывание
      {
        //Двойное нажатие или охрана выкл, двигатель работает, раскладываем зеркала 
        if (Jeep_Batt < 11.0)
        {  
          Serial.println(F("---Open mirrors ! ---"));     
          digitalWrite(Mirrors_Open, HIGH);
        }
        else
        {
          Serial.println(F("--- Отмена расскладывания. Напряжение аккумулятора менее 12В ! ---"));
        }
        my_mirrors = millis();
        Jeep_Mirrors_Open = 7;
      }
    }
    // Закрывание зеркал
    // в движении не складыванием!
    if ((keyState == 0x81 and Jeep_RPM > 500) and (Jeep_Mirrors_Close != 0))
    {
      // двигатель работает, складывание запрещено
      Serial.println(F("---В движении складывать зеркала нельзя ! ---")); 
      Jeep_Mirrors_Close = 0;
    }
    if (Jeep_Mirrors_Close != 0)
    {
      // 01 - одиночная постановка на охрану, 02 - двойная постановка на охрану
      if ( Jeep_Mirrors_Close == 1)// проверяем Stage 01
      {
        // На охране
        // Пауза. ждем второго нажатия кнопки
        if (millis() - my_mirrors >= 10000)
        {
          // время ожидания вышло. Сброс.
          Jeep_Mirrors_Close = 0; 
          my_mirrors = millis();
          Serial.println(F("---Mirrors Close Time Reset = ---"));
        }
        else
        {
          // Ожидаем второго нажатия
          Serial.println(F("---Mirrors Close Pause: Wait = ---"));
        }
      }
      if ( Jeep_Mirrors_Close == 2)// проверяем Stage 02
      {
        //Двойное нажатие - складываем зеркала 
        if (Jeep_Batt < 11.0)
        { 
          Serial.println(F("---Close mirrors !!!---"));     
          digitalWrite(Mirrors_Close, HIGH);
        }
        else
        {
          Serial.println(F("--- Отмена складывания. Напряжение аккумулятора менее 12В ! ---"));
        }
        my_mirrors = millis();
        Jeep_Mirrors_Close = 7;
      }
    }
    // завершение расскладывания
    if ( Jeep_Mirrors_Open == 7)// 07 - временная пауза. завершение процесса
    {
      //врееменная пауза. завершение процесса       
      if (millis() - my_mirrors >= 1000)
      {
        // время ожидания вышло.
        digitalWrite(Mirrors_Open, LOW);
        Jeep_Mirrors_Open = 0;
       }
    }
    // завершение складывания
    if ( Jeep_Mirrors_Close == 7)// 07 - временная пауза. завершение процесса
    {
      //временная пауза. завершение процесса       
      if (millis() - my_mirrors >= 1000)
      {
        // время ожидания вышло.
        digitalWrite(Mirrors_Close, LOW);
        Jeep_Mirrors_Close = 0;
      }
    }    
  }
  else
  {
    Jeep_Mirrors_Close = 0;
    Jeep_Mirrors_Open = 0;
    // на всякий случай, проверяем что расскладывание выкл
    if (digitalRead(Mirrors_Open) == HIGH)
    {
      digitalWrite(Mirrors_Open, LOW);
    }
    // на всякий случай, проверяем что складывание выкл
    if (digitalRead(Mirrors_Close) == HIGH)
    {
      digitalWrite(Mirrors_Close, LOW);
    }
  }
}

void Check_RKE_Button()
{
  //RKE_Trunk_Button Кнопка открытия багажника
  if (RKE_Trunk_Button_flag == true)
  {
    // Fobik Trunk Key Enabled	
    if (digitalRead(RKE_Trunk_Button) == LOW)
    { 
      Serial.println(F("---Fobik Key Enabled = Trunk OUT ON ---"));
      digitalWrite(RKE_Trunk_Button, HIGH);
    }
    else
    {
      Serial.println(F("---Fobik Key Enabled = Trunk OUT OFF ---"));
      digitalWrite(RKE_Trunk_Button, LOW);
    }
    RKE_Trunk_Button_flag = false;
  }
  
  //RKE_Alarm_ON Постановка на охрану
  if (RKE_Alarm_ON_flag == true)
  {
    // Fobik Key Enabled Alarm ON
    Alarm_ON = true;
    Alarm_OFF = false;
    if (BEEP == true)
    { 
      beep();// Beep
    }
    Serial.println(F("---Fobik Key Enabled = Alarm ON ---"));	  
    Jeep_Mirrors_Open = 0;// зеркала открывать не нужно
    Jeep_Mirrors_Close += 1;// зеркала складывать нужно, след шаг
    if (Jeep_Mirrors_Close == 1)
    { 
      my_mirrors = millis();// временная задержка
    }	  
    RKE_Alarm_ON_flag = false;
  }

  //RKE_Alarm_OFF Снятие с охраны
  if (RKE_Alarm_OFF_flag == true)
  {
    // Fobik Key Enabled Alarm OFF 
    Alarm_ON = false;
    Alarm_OFF = true;  
    Serial.println(F("---Fobik Key Enabled = Alarm OFF ---"));
    Jeep_Mirrors_Close = 0;// зеркала складывать не нужно
    Jeep_Mirrors_Open += 1;// зеркала открывать нужно, след шаг
    //if (Jeep_Mirrors_Open == 1)
    //{ 
      //my_mirrors = millis();// временная задержка 
    //}  
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
  // Проверяем рулевые кнопки
  if (Steering_Wheel_1_flag == true)
  {
    // длительное нажатие кнопки
    if (Temp_Button_SW1 == 0x01)
    {
      // Центральная правая подрулевая кнопка
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
        beep();//Beep
      }
    }

    if (Temp_Button_SW1 == 0x20)
    { 
      // Центральная левая подрулевая кнопка
      if (digitalRead(Steering_Wheel_1) != HIGH)
      { 
        Serial.println(F("---Steering Wheel Key Enabled = OUT ON ---"));
        digitalWrite(Steering_Wheel_1, HIGH);
      }
      else
      {
        Serial.println(F("---Steering Wheel Key Enabled = OUT OFF ---"));
        digitalWrite(Steering_Wheel_1, LOW);
      }
      beep();// Beep
    }

    if (Temp_Button_SW1 == 0x08)
    {
      // Верхняя левая подрулевая кнопка
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
      // Нижняя правая подрулевая кнопка
      if (Jeep_Demo_FOG == 0)
      {
        Jeep_Demo_FOG = 1;
        Serial.println(F("---Demo Fog Enabled ---"));
        beep();// Beep
      }
      else
      {
        Jeep_Demo_FOG = 0;
        Serial.println(F("---Demo Fog Disabled ---"));
        beep();// Beep
        delay(100);
        beep();// Beep
      }
    }

    if (Temp_Button_SW1 == 0x10)
    {
      // Нижняя левая подрулевая кнопка
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
    // включены штатно туманки - выключаем демо и подсветку поворота
    if (FrontFogON == true)
    {
      kill_all_fog();
      Jeep_Demo_FOG = 0;
    }
    else
    { 
      // подсветка поворота
      if ((Jeep_Demo_FOG == 0) && (Engine_Run == true) && (FrontFogON == false) && (Jeep_Speed <= 9)  && (Jeep_Speed != 0) && (Jeep_Gear == 0x44))
      {
        if ( RightFog == true )
        {
          //Fog right enable
          if (digitalRead(EnableFogRight) != HIGH)
          { 
            digitalWrite(EnableFogRight, HIGH);
            Serial.println(F("---Right Fog ON---"));
          } 
        }
        else
        {
          //Fog right disable
          if (digitalRead(EnableFogRight) == HIGH)
          { 
            digitalWrite(EnableFogRight, LOW);
            Serial.println(F("---Right Fog OFF---"));
          }  
        }

        if ( LeftFog == true )
        {
          //Fog left enable
          if (digitalRead(EnableFogLeft) != HIGH)
          { 
            digitalWrite(EnableFogLeft, HIGH);
            Serial.println(F("---Left Fog ON---"));
          }   
        }
        else
        {
          //Fog left disable
          if (digitalRead(EnableFogLeft) == HIGH)
          { 
            digitalWrite(EnableFogLeft, LOW);
            Serial.println(F("---Left Fog OFF---")); 
          }
        } 
      }
      else
      {
        kill_all_fog(); 
      } 
      // Демо
      if ((Jeep_Demo_FOG != 0) && (Engine_Run == true) && (FrontFogON == false))
      {
        if (Jeep_Demo_FOG == 1)
        {
          //step 1 Right fog
          if (digitalRead(EnableFogLeft) == HIGH)
          { 
            digitalWrite(EnableFogLeft, LOW);
            Serial.println(F("---Demo Left Fog OFF---")); 
          }
          if (digitalRead(EnableFogRight) != HIGH)
          { 
            digitalWrite(EnableFogRight, HIGH);
            Serial.println(F("---Demo Right Fog ON---"));
          } 
        }
        if (Jeep_Demo_FOG == 2)
        {
          //step 2 Left Fog
          if (digitalRead(EnableFogRight) == HIGH)
          { 
            digitalWrite(EnableFogRight, LOW);
            Serial.println(F("---Demo Right Fog OFF---"));  
          }
          if (digitalRead(EnableFogLeft) != HIGH)
          { 
            digitalWrite(EnableFogLeft, HIGH);
            Serial.println(F("---Demo Left Fog ON---"));
          }
        }
        Jeep_Demo_FOG += 1;
        if (Jeep_Demo_FOG > 2)
        {
          //next step Demo FOG
          Jeep_Demo_FOG = 1;
        }
      }
      else
      {
        Jeep_Demo_FOG = 0;
        kill_all_fog(); 
      }
      //Двигатель остановлен - отключаем демо и туманки
      if (Engine_Run == false)
      {
        kill_all_fog();
        Jeep_Demo_FOG = 0;
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
          canSend(0x02E, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00); // включаем левый попогрей
        }
        if (Jeep_HeatSeat_Status_2 == 0x00)
        {
          // Подогрев правого сидения выключен
          canSend(0x02E, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00); // включаем правый попогрей
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
      Jeep_RPM = ( parameters[0] << 8 ) + parameters[1];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Speed, RPM and other: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }      
      break;

    case 0x003:
      Jeep_Gear = parameters[4];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Gear pasition and other: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
	
    case 0x015:
      Jeep_Batt = parameters[1] / 10.0;
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
        Jeep_Demo_FOG = 0; // Demo_Fog - disable
        // Front Fog enable. Left and right fogs disable
        if (Settings_FOG == true)
        {
          kill_all_fog();
        }		
      }
      else
      {
        FrontFogON = false; // FrontFogON - Front fog OFF - false
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
	  
    case 0x1AA:
      Jeep_Rain_Sensor = parameters[0];
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Rain Sensor: ");
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
      //Jeep_RKE_Code = (( parameters[4] << 8 ) + parameters[5]);
      
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
      Settings_HEAT_SEAT == true;// HSM installed
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Ident HSM: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;

    case 0x421:
      Settings_Rain_Sensor == true;// LRSM installed
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Ident LRSM: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
      break;
      
    case 0x43F:
      Settings_VES == false;// VES installed
      if (CAN_LOGS == true)
      {
        Serial.print("0x");
        Serial.print(packetID, HEX);
        Serial.print(" Ident VES: ");
        Serial.print(packetSize);
        for (uint8_t x = 0; x < packetSize; x++)
        {
          Serial.print(" 0x"); Serial.print(parameters[x], HEX);
        }
        Serial.println();
      }
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
}

void beep()
{
  // Beep
  // to do: check parameter Jeep_Hasards
  //canSend(0x11D, Jeep_Hasards, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
  canSend(0x11D, 0x80, Jeep_Wiper, 0x00, 0x00, 0x00, 0x00);
}

void kill_all_fog()
{
  // Disable left and right fog
  if (digitalRead(EnableFogLeft) == HIGH)
  { 
    digitalWrite(EnableFogLeft, LOW);
    Serial.println(F("---Kill All Fog: Left---")); 
  }
  if (digitalRead(EnableFogRight) == HIGH)
  { 
    digitalWrite(EnableFogRight, LOW);
    Serial.println(F("---Kill All Fog: Right---"));
  } 
}

void Check_Alarm()
{
  // постанова на охрану
  if ( Alarm_ON == true )
  {
	  // Постановка на охрану
    // отключаем выход Steering_Wheel_1
    if (digitalRead(Steering_Wheel_1) == HIGH)
    { 
      Serial.println(F("---Alarm ON: Disable Units Steering Wheel ---"));
      digitalWrite(Steering_Wheel_1, LOW);
    }
    // прочее при постановке на охрану
	  Alarm_ON = false;// завершение всего прочего при постановке на охрану
  }

  // снятие с охраны
  if ( Alarm_OFF == true )
  {
	  // Снятие с охраны
    // прочее при снятии с охраны
	  Alarm_OFF = false;// завершение всего прочего при снятии с охраны
  }
}

void Check_Rain()
{
  // Авто-дворники 
  if ( Settings_Rain_Sensor == true )
  {
    if ((Settings_HOT_TEMP == true) and (Jeep_Temp_Outdoor > 0x50)) // по температурному датчику
    {
      // ок, проверяем данные датчика дождя
      if (Engine_Run == true and (Jeep_Rain_Sensor == 0x20) or (Jeep_Rain_Sensor == 0x21))
      {
        // Stage 1: редкие взмахи передних дворников
        if (millis() - my_wiper_mirrors >= 30000)
        {
          my_wiper_mirrors = millis();
          canSend(0x1F8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
          Serial.println(F("---Rear Wiper Rain stage 1 = ---"));
        }
      }
      if (Engine_Run == true and (Jeep_Rain_Sensor == 0x40) or (Jeep_Rain_Sensor == 0x41))
      {
        // Stage 2: быстрые взмахи передних дворников
        if (millis() - my_wiper_mirrors >= 20000)
        {
          my_wiper_mirrors = millis();
          canSend(0x1F8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
          Serial.println(F("---Rear Wiper Rain stage 2 = ---"));
        }
      }
      if (Engine_Run == true and (Jeep_Rain_Sensor == 0x60) or (Jeep_Rain_Sensor == 0x61))
      {
        // Stage 3: очень-быстрые взмахи передних дворников
        if (millis() - my_wiper_mirrors >= 10000)
        {
          my_wiper_mirrors = millis();
          canSend(0x1F8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
          Serial.println(F("---Rear Wiper Rain stage 3 = ---"));
        }
      }
    
      if ( Jeep_Rain_Sensor == 0x01 || Jeep_Rain_Sensor == 0x21 || Jeep_Rain_Sensor == 0x41 || Jeep_Rain_Sensor == 0x61 ) //Check night
      {
        //Night 
        //Serial.println(F(" Rain Sensor - Night"));
        //canSend(0x1A2, 0x00, 0xEF, 0x16, 0xFF, 0x00, 0x00); 
      }
      if ( Jeep_Rain_Sensor == 0x00 || Jeep_Rain_Sensor == 0x20 || Jeep_Rain_Sensor == 0x40 || Jeep_Rain_Sensor == 0x60 ) //Check day
      {
        //Day
        //Serial.println(F(" Rain Sensos - Day"));
        //canSend(0x1A2, 0x00, 0x6E, 0x16, 0xFF, 0x00, 0x00); 
      }
    }
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
      canSend(0x370, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);        
    }
    if ( RX == 'u' || RX == 'U' ) //
    {
       // wake on can 
       canSend(0x7FF, 0x00);        
    }
    if ( RX == 'd' || RX == 'D' ) //
    {
      // demo fog
      Jeep_Demo_FOG = 1;
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
