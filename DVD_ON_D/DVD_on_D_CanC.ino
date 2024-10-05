#include "mcp_can.h"
#include <SPI.h>

unsigned long rxId;
byte len;
byte rxBuf[8];
char msgString[128];

byte txBuf0[] = {0x00, 0x00, 0x80, 0x20, 0x50, 0xFF, 0x0C}; // включен P, мотор заведён, тормоз отпущен.
byte txBuf1[] = {0x00, 0x00, 0x84, 0x21, 0x44, 0xFF, 0x0C}; // включен D, мотор заведён, тормоз нажат.

MCP_CAN CAN0(10); // на десятом пине CS от адаптера "в машину".
MCP_CAN CAN1(9);  // на десятом пине CS от адаптера "в радио".

void setup()
{
  Serial.begin(115200);
  // инициализация CAN1 bus, baudrate: 125k@8MHz Если шина медленная, на более старых моделях,
  // надо вместо CAN_125KBPS написать CAN_83K3BPS - 83,3Kbit/s, но там хз, может и коды другие.
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
  Serial.print("CAN0: Init OK!\r\n");
  CAN0.setMode(MCP_NORMAL);
  } else Serial.print("CAN0: Init Fail!!!\r\n");
  
  // инициализация CAN1 bus, baudrate: 125k@8MHz Если шина медленная, на более старых моделях,
  // надо вместо CAN_125KBPS написать CAN_83K3BPS - 83,3Kbit/s но там хз, может и коды другие.
  if(CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
  Serial.print("CAN1: Init OK!\r\n");
  CAN1.setMode(MCP_NORMAL);
  } else Serial.print("CAN1: Init Fail!!!\r\n");
}

void loop(){  
  if(!digitalRead(2)){          // на втором пине INT от адаптера "в машину". Считываем, если он в LOW.
    CAN0.readMsgBuf(&rxId, &len, rxBuf);   // читаем из шины и посмотрим, пропускать ли дальше в радио.
    if(rxId == 0x20E){                // в сообщениях с этим ID передаются данные о положении ручки.
     if(rxBuf[2]==txBuf1[2] &&        // сравниваем не все байты, а только постоянные при
        rxBuf[4]==txBuf1[4] &&        // положении ручки в D. Другие меняются на 
        rxBuf[5]==txBuf1[5] &&        // заглушенном двигателе и отпущенной педали тормоза.
        rxBuf[6]==txBuf1[6]){
           memcpy(rxBuf, txBuf0, 7);  // заменяем D на P
      }
    }
    CAN1.sendMsgBuf(rxId, 0, len, rxBuf); // отсылаем в радио
  }
  if(!digitalRead(3)){          // на третьем пине INT от адаптера "в радио". Считываем, если он в LOW.
    CAN1.readMsgBuf(&rxId, &len, rxBuf); 
    CAN0.sendMsgBuf(rxId, 0, len, rxBuf); 
  }
}
