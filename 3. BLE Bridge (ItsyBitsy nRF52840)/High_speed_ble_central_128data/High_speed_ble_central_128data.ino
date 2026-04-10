/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today 
  in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
   This sketch demonstrate the central API(). A additional bluefruit
   that has bleuart as peripheral is required for the demo.
*/
#include <bluefruit.h>

BLEClientUart clientUart; // bleuart client
int rs232_datalength = 128;
int ble_datalength = 128;
uint32_t rx_count = 0;
uint8_t data_rs232_rx[247] = { 0 };
uint8_t data_ble_rx[247] = { 0 };
uint16_t serial_frame_idx = 0;
const char kTargetName[] = "Shutong";

uint32_t gui_in_bytes_total = 0;
uint32_t gui_in_bytes_sec = 0;
uint32_t ble_out_bytes_total = 0;
uint32_t ble_out_bytes_sec = 0;
uint32_t gui_to_ble_frames_total = 0;
uint32_t gui_to_ble_frames_sec = 0;
uint32_t dropped_gui_frames_total = 0;
uint32_t dropped_gui_frames_sec = 0;
uint32_t frame_sync_resets_total = 0;
uint32_t frame_sync_resets_sec = 0;

uint32_t ble_in_bytes_total = 0;
uint32_t ble_in_bytes_sec = 0;
uint32_t gui_out_bytes_total = 0;
uint32_t gui_out_bytes_sec = 0;
uint32_t ble_to_gui_packets_total = 0;
uint32_t ble_to_gui_packets_sec = 0;

uint8_t last_gui_to_ble_head[3] = {0};
uint8_t last_ble_to_gui_head[3] = {0};

void print_link_status();
void print_hex_byte(uint8_t value);
void setup()
{
  Serial.begin(115200);
  // while ( !Serial );  // ✅ 等待串口连接（重要！）

  Serial.println("=== Central BLE UART Starting ===");

  delay(100);   // 让串口有时间启动
//  Serial.println("Bluefruit52 Central BLEUART Example");
//  Serial.println("-----------------------------------\n");

  // Config the connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Bluefruit.setName("Bluefruit52 Central");

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Don't use active scan
     - Start(timeout) with timeout = 0 will scan forever (until connected)
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
   Callback invoked when scanner pick up an advertising data
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  char devname[32] = { 0 };

  const uint8_t* adv_data = report->data.p_data;
  uint8_t adv_len = report->data.len;

  for (uint8_t index = 0; index < adv_len;) {
    uint8_t field_len = adv_data[index];
    if (field_len == 0) break;
    if ((uint16_t)index + field_len >= adv_len) break;

    uint8_t type = adv_data[index + 1];

    if (type == 0x08 || type == 0x09) {
      uint8_t name_len = field_len - 1;
      if (name_len >= sizeof(devname)) name_len = sizeof(devname) - 1;
      memcpy(devname, &adv_data[index + 2], name_len);
      devname[name_len] = '\0';
      break;
    }

    index += field_len + 1;
  }

  Serial.print("Found device: ");
  Serial.println(devname);  // ✅ 打印所有扫描到的设备名

  if (strcmp(devname, kTargetName) == 0) {   // 直接用名字匹配
    Serial.println("Match found, connecting...");
    Bluefruit.Central.connect(report);
  } else {
    Bluefruit.Scanner.resume();
  }

}




/**
   Callback invoked when an connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle)
{
//  Serial.println("Connected");

//  Serial.print("Discovering BLE Uart Service ... ");
  if ( clientUart.discover(conn_handle) )
  {
//    Serial.println("Found it");
//    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();
//    Serial.println("Ready to receive from peripheral");
  } else
  {
//    Serial.println("Found NONE");
    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
//  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

/**
   Callback invoked when uart received data
   @param uart_svc Reference object to the service where the data
   arrived. In this example it is clientUart
*/
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  ble_datalength = uart_svc.available();
  if (ble_datalength <= 0) return;
  if (ble_datalength > (int)sizeof(data_ble_rx)) ble_datalength = sizeof(data_ble_rx);
  size_t n = uart_svc.readBytes((char*)data_ble_rx, (size_t)ble_datalength);
  if (n == 0) return;
  ble_in_bytes_total += (uint32_t)n;
  ble_in_bytes_sec += (uint32_t)n;
  ble_to_gui_packets_total++;
  ble_to_gui_packets_sec++;
  if (n >= 3)
  {
    last_ble_to_gui_head[0] = data_ble_rx[0];
    last_ble_to_gui_head[1] = data_ble_rx[1];
    last_ble_to_gui_head[2] = data_ble_rx[2];
  }
  size_t nw = Serial.write(data_ble_rx, n);
  gui_out_bytes_total += (uint32_t)nw;
  gui_out_bytes_sec += (uint32_t)nw;
  // Serial.println(data_ble_rx[2],DEC);
//  Serial.print("Received from peripheral: ");
//  for (int i = 0; i < ble_datalength; i++)
//  {
//    Serial.print(data_ble_rx[i],DEC);
//    Serial.print("  ");
//  }  
//  Serial.println("");
}

void loop()
{
  // Discovered means in working state
  // Get Serial input and send to Peripheral
  while (Serial.available())
  {
    uint8_t byte_in = (uint8_t)Serial.read();
    gui_in_bytes_total++;
    gui_in_bytes_sec++;

    if (serial_frame_idx == 0)
    {
      if (byte_in != 0xA5) continue;
      data_rs232_rx[serial_frame_idx++] = byte_in;
      continue;
    }

    if (serial_frame_idx == 1)
    {
      if (byte_in != 0x5A && byte_in != 0x5F)
      {
        frame_sync_resets_total++;
        frame_sync_resets_sec++;
        serial_frame_idx = 0;
        continue;
      }
      data_rs232_rx[serial_frame_idx++] = byte_in;
      continue;
    }

    if (serial_frame_idx == 2)
    {
      if (byte_in != (uint8_t)rs232_datalength)
      {
        frame_sync_resets_total++;
        frame_sync_resets_sec++;
        serial_frame_idx = 0;
        continue;
      }
      data_rs232_rx[serial_frame_idx++] = byte_in;
      continue;
    }

    data_rs232_rx[serial_frame_idx++] = byte_in;

    if (serial_frame_idx >= (uint16_t)rs232_datalength)
    {
      last_gui_to_ble_head[0] = data_rs232_rx[0];
      last_gui_to_ble_head[1] = data_rs232_rx[1];
      last_gui_to_ble_head[2] = data_rs232_rx[2];
      if (Bluefruit.Central.connected() && clientUart.discovered())
      {
        size_t sent = clientUart.write(data_rs232_rx, (size_t)rs232_datalength);
        ble_out_bytes_total += (uint32_t)sent;
        ble_out_bytes_sec += (uint32_t)sent;
        gui_to_ble_frames_total++;
        gui_to_ble_frames_sec++;
      }
      else
      {
        dropped_gui_frames_total++;
        dropped_gui_frames_sec++;
      }
      serial_frame_idx = 0;
    }
  }
//  print_link_status();
//  data_rs232_rx[0] = 165;
//  data_rs232_rx[1] = 90;
//  data_rs232_rx[2] = data_rs232_rx[2] + 1;
//  if ( Bluefruit.Central.connected() && clientUart.discovered() )
//  {
//    clientUart.write(data_rs232_rx, rs232_datalength);
//  }
//  delay(1000);
}

void print_link_status()
{
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < 1000) return;
  last_ms = now;

  Serial.print("[STAT][CENT] conn=");
  Serial.print(Bluefruit.Central.connected() ? 1 : 0);
  Serial.print(" disc=");
  Serial.print(clientUart.discovered() ? 1 : 0);
  Serial.print(" idx=");
  Serial.print(serial_frame_idx);
  Serial.print(" | GUI->BLE B/s=");
  Serial.print(gui_in_bytes_sec);
  Serial.print(" BLE_TX_B/s=");
  Serial.print(ble_out_bytes_sec);
  Serial.print(" frm/s=");
  Serial.print(gui_to_ble_frames_sec);
  Serial.print(" drop/s=");
  Serial.print(dropped_gui_frames_sec);
  Serial.print(" syncReset/s=");
  Serial.print(frame_sync_resets_sec);
  Serial.print(" head=");
  print_hex_byte(last_gui_to_ble_head[0]);
  Serial.print(" ");
  print_hex_byte(last_gui_to_ble_head[1]);
  Serial.print(" ");
  print_hex_byte(last_gui_to_ble_head[2]);
  Serial.print(" | BLE->GUI B/s=");
  Serial.print(ble_in_bytes_sec);
  Serial.print(" GUI_TX_B/s=");
  Serial.print(gui_out_bytes_sec);
  Serial.print(" pkt/s=");
  Serial.print(ble_to_gui_packets_sec);
  Serial.print(" head=");
  print_hex_byte(last_ble_to_gui_head[0]);
  Serial.print(" ");
  print_hex_byte(last_ble_to_gui_head[1]);
  Serial.print(" ");
  print_hex_byte(last_ble_to_gui_head[2]);
  Serial.println();

  gui_in_bytes_sec = 0;
  ble_out_bytes_sec = 0;
  gui_to_ble_frames_sec = 0;
  dropped_gui_frames_sec = 0;
  frame_sync_resets_sec = 0;
  ble_in_bytes_sec = 0;
  gui_out_bytes_sec = 0;
  ble_to_gui_packets_sec = 0;
}

void print_hex_byte(uint8_t value)
{
  if (value < 0x10) Serial.print("0");
  Serial.print(value, HEX);
}
