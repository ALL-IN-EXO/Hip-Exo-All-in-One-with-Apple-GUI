

/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <bluefruit.h>

/* Best result is
    - 8.74 KB/s with 20 ms, MTU = 23
    - 23.62 KB/s with 7.5 ms, MTU = 23
    - 47.85 KB/s with 15 ms, MTU = 247
*/

int rs232_datalength = 128;
int ble_datalength = 128;
uint8_t data_rs232_rx[247] = {0};
uint8_t data_ble_rx[247] = {0};
uint16_t serial1_frame_idx = 0;

// Link statistics
uint32_t uart_in_bytes_total = 0;
uint32_t uart_in_bytes_sec = 0;
uint32_t ble_out_bytes_total = 0;
uint32_t ble_out_bytes_sec = 0;
uint32_t uart_to_ble_frames_total = 0;
uint32_t uart_to_ble_frames_sec = 0;
uint32_t dropped_uart_frames_total = 0;
uint32_t dropped_uart_frames_sec = 0;
uint32_t frame_sync_resets_total = 0;
uint32_t frame_sync_resets_sec = 0;

uint32_t ble_in_bytes_total = 0;
uint32_t ble_in_bytes_sec = 0;
uint32_t uart_out_bytes_total = 0;
uint32_t uart_out_bytes_sec = 0;
uint32_t ble_to_uart_packets_total = 0;
uint32_t ble_to_uart_packets_sec = 0;
uint8_t last_uart_to_ble_head[3] = {0};
uint8_t last_ble_to_uart_head[3] = {0};
// Number of packet to sent
// actualy number of bytes depends on the MTU of the connection
#define PACKET_NUM    1000

BLEDis bledis;
BLEUart bleuart;

uint32_t rxCount = 0;
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;

void print_link_status();
void print_hex_byte(uint8_t value);

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial1.begin(115200);
  //while ( !Serial )
  delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Throughput Example");
  Serial.println("------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Shutong");         // 设置名称
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12); // 7.5 - 15 ms

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  bleuart.setRxCallback(bleuart_rx_callback);
  bleuart.setNotifyCallback(bleuart_notify_callback);




  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println("Connected");

  // request PHY changed to 2MB
  Serial.println("Request to change PHY");
  conn->requestPHY();

  // request to update data length
  Serial.println("Request to change Data Length");
  conn->requestDataLengthUpdate();

  // request mtu exchange
  Serial.println("Request to change MTU");
  conn->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  //conn->requestConnectionParameter(6); // in unit of 1.25

  // delay a bit for all the request to complete
  delay(1000);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;
  ble_datalength = bleuart.available();
  if (ble_datalength <= 0) return;
  if (ble_datalength > (int)sizeof(data_ble_rx)) ble_datalength = sizeof(data_ble_rx);
  size_t n = bleuart.readBytes((char*)data_ble_rx, (size_t)ble_datalength);
  if (n == 0) return;
  ble_in_bytes_total += (uint32_t)n;
  ble_in_bytes_sec += (uint32_t)n;
  ble_to_uart_packets_total++;
  ble_to_uart_packets_sec++;
  if (n >= 3)
  {
    last_ble_to_uart_head[0] = data_ble_rx[0];
    last_ble_to_uart_head[1] = data_ble_rx[1];
    last_ble_to_uart_head[2] = data_ble_rx[2];
  }
  //  for(int i=0;i<ble_datalength;i++)
  //  {
  //      Serial.print(data_ble_rx[i], DEC);
  //      Serial.print("  ");
  //  }
  //      Serial.println("");
  size_t nw = Serial1.write(data_ble_rx, n);
  uart_out_bytes_total += (uint32_t)nw;
  uart_out_bytes_sec += (uint32_t)nw;
//  Serial.println("Received from central: ");
//  for (int i = 0; i < ble_datalength; i++) {
//    Serial.print(data_ble_rx[i], DEC);
//    Serial.print("  ");
//  }
//  Serial.println("");
}

void bleuart_notify_callback(uint16_t conn_hdl, bool enabled)
{
  if ( enabled )
  {
    Serial.println("notify_callback");
  }
}
void loop(void)
{
  while (Serial1.available())
  {
    uint8_t byte_in = (uint8_t)Serial1.read();
    uart_in_bytes_total++;
    uart_in_bytes_sec++;

    if (serial1_frame_idx == 0)
    {
      if (byte_in != 0xA5) continue;
      data_rs232_rx[serial1_frame_idx++] = byte_in;
      continue;
    }

    if (serial1_frame_idx == 1)
    {
      if (byte_in != 0x5A && byte_in != 0x5F)
      {
        frame_sync_resets_total++;
        frame_sync_resets_sec++;
        serial1_frame_idx = 0;
        continue;
      }
      data_rs232_rx[serial1_frame_idx++] = byte_in;
      continue;
    }

    if (serial1_frame_idx == 2)
    {
      if (byte_in != (uint8_t)rs232_datalength)
      {
        frame_sync_resets_total++;
        frame_sync_resets_sec++;
        serial1_frame_idx = 0;
        continue;
      }
      data_rs232_rx[serial1_frame_idx++] = byte_in;
      continue;
    }

    data_rs232_rx[serial1_frame_idx++] = byte_in;

    if (serial1_frame_idx >= (uint16_t)rs232_datalength)
    {
      last_uart_to_ble_head[0] = data_rs232_rx[0];
      last_uart_to_ble_head[1] = data_rs232_rx[1];
      last_uart_to_ble_head[2] = data_rs232_rx[2];
      if (Bluefruit.connected() && bleuart.notifyEnabled())
      {
        size_t sent = bleuart.write(data_rs232_rx, (size_t)rs232_datalength);
        ble_out_bytes_total += (uint32_t)sent;
        ble_out_bytes_sec += (uint32_t)sent;
        uart_to_ble_frames_total++;
        uart_to_ble_frames_sec++;
      }
      else
      {
        dropped_uart_frames_total++;
        dropped_uart_frames_sec++;
      }
      serial1_frame_idx = 0;
    }
  }
  print_link_status();
  //    delay(500);
  //    data_ble_rx[0]=165;
  //    data_ble_rx[1]=90;
  //    data_ble_rx[2]=data_ble_rx[2]+1;
  //    Serial1.write(data_ble_rx, 20);
  //  data_rs232_rx[0] = 165;
  //  data_rs232_rx[1] = 90;
  //  data_rs232_rx[2] = data_rs232_rx[2] + 1;
  //  if (Bluefruit.connected() && bleuart.notifyEnabled())
  //  {
  //    bleuart.write(data_rs232_rx, rs232_datalength);
  //  }
  //delay(1); don't use delay
}

void print_link_status()
{
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < 1000) return;
  last_ms = now;

  Serial.print("[STAT][PRPH] conn=");
  Serial.print(Bluefruit.connected() ? 1 : 0);
  Serial.print(" notify=");
  Serial.print(bleuart.notifyEnabled() ? 1 : 0);
  Serial.print(" idx=");
  Serial.print(serial1_frame_idx);
  Serial.print(" | UART->BLE B/s=");
  Serial.print(uart_in_bytes_sec);
  Serial.print(" BLE_TX_B/s=");
  Serial.print(ble_out_bytes_sec);
  Serial.print(" frm/s=");
  Serial.print(uart_to_ble_frames_sec);
  Serial.print(" drop/s=");
  Serial.print(dropped_uart_frames_sec);
  Serial.print(" syncReset/s=");
  Serial.print(frame_sync_resets_sec);
  Serial.print(" head=");
  print_hex_byte(last_uart_to_ble_head[0]);
  Serial.print(" ");
  print_hex_byte(last_uart_to_ble_head[1]);
  Serial.print(" ");
  print_hex_byte(last_uart_to_ble_head[2]);
  Serial.print(" | BLE->UART B/s=");
  Serial.print(ble_in_bytes_sec);
  Serial.print(" UART_TX_B/s=");
  Serial.print(uart_out_bytes_sec);
  Serial.print(" pkt/s=");
  Serial.print(ble_to_uart_packets_sec);
  Serial.print(" head=");
  print_hex_byte(last_ble_to_uart_head[0]);
  Serial.print(" ");
  print_hex_byte(last_ble_to_uart_head[1]);
  Serial.print(" ");
  print_hex_byte(last_ble_to_uart_head[2]);
  Serial.println();

  uart_in_bytes_sec = 0;
  ble_out_bytes_sec = 0;
  uart_to_ble_frames_sec = 0;
  dropped_uart_frames_sec = 0;
  frame_sync_resets_sec = 0;
  ble_in_bytes_sec = 0;
  uart_out_bytes_sec = 0;
  ble_to_uart_packets_sec = 0;
}

void print_hex_byte(uint8_t value)
{
  if (value < 0x10) Serial.print("0");
  Serial.print(value, HEX);
}
