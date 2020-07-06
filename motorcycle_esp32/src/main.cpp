#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size


const int OUT0 = 26;

void setup() {
  Serial.begin(9600);
  pinMode(OUT0, OUTPUT);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_0;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}

void loop() {

 CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  
  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      printf("New standard frame");
    }
    else {
      printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        printf("0x%02X ", rx_frame.data.u8[i]);
      }
      printf("\n");
    }

    if(rx_frame.MsgID == 0x01DD0001)
    {
      if(rx_frame.data.u8[0] == 0x00)
      {
        digitalWrite(OUT0, HIGH);
      }
      else
      {
        digitalWrite(OUT0, LOW);
      }
      
    }


  }

  
  // Send CAN Message
  if (currentMillis - previousMillis >= interval && false) {
    previousMillis = currentMillis;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_ext;
    tx_frame.MsgID = 0x18E54024;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0xFC;
    tx_frame.data.u8[1] = 0x8C;
    tx_frame.data.u8[2] = 0x0;
    tx_frame.data.u8[3] = 0x62;
    tx_frame.data.u8[4] = 0x0C;
    tx_frame.data.u8[5] = 0x02;
    tx_frame.data.u8[6] = 0xFF; 
    tx_frame.data.u8[7] = 0xFF;
    ESP32Can.CANWriteFrame(&tx_frame);
     printf("command sent\n");
  }
}
