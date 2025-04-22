/*************************************************************************
 *
 * GAMEBOY PRINTER EMULATION PROJECT V3.2.1 (Arduino)
 * Copyright (C) 2022 Brian Khuu
 *
 * PURPOSE: To capture gameboy printer images without a gameboy printer
 *          via the arduino platform. (Tested on the arduino nano)
 *          This version is to investigate gameboy behaviour.
 *          This was originally started on 2017-4-6 but updated on 2020-08-16
 * LICENCE:
 *   This file is part of Arduino Gameboy Printer Emulator.
 *
 *   Arduino Gameboy Printer Emulator is free software:
 *   you can redistribute it and/or modify it under the terms of the
 *   GNU General Public License as published by the Free Software Foundation,
 *   either version 3 of the License, or (at your option) any later version.
 *
 *   Arduino Gameboy Printer Emulator is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Arduino Gameboy Printer Emulator.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

// See /WEBUSB.md for details
#if USB_VERSION == 0x210
#include <WebUSB.h>
WebUSB WebUSBSerial(1, "herrzatacke.github.io/gb-printer-web/#/webusb");
#define Serial WebUSBSerial
#endif

#define GAME_BOY_PRINTER_MODE      true   // to use with https://github.com/Mraulio/GBCamera-Android-Manager and https://github.com/Raphael-Boichot/PC-to-Game-Boy-Printer-interface
#define GBP_OUTPUT_RAW_PACKETS     true   // by default, packets are parsed. if enabled, output will change to raw data packets for parsing and decompressing later
#define GBP_USE_PARSE_DECOMPRESSOR false  // embedded decompressor can be enabled for use with parse mode but it requires fast hardware (SAMD21, SAMD51, ESP8266, ESP32)

#include <stdint.h>  // uint8_t
#include <stddef.h>  // size_t

#include "gameboy_printer_protocol.h"
#include "gbp_serial_io.h"
#include "gbp_decoder.h"

#if GBP_OUTPUT_RAW_PACKETS
#define GBP_FEATURE_PACKET_CAPTURE_MODE
#else
#define GBP_FEATURE_PARSE_PACKET_MODE
#if GBP_USE_PARSE_DECOMPRESSOR
#define GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
#endif
#endif

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
#include "gbp_pkt.h"
#endif

// Includo le librerie per il display TFT SPI 1.8"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <SPIFFS.h>

/* Gameboy Link Cable Mapping to Arduino Pin */
// Note: Serial Clock Pin must be attached to an interrupt pin of the arduino
//  ___________
// |  6  4  2  |
//  \_5__3__1_/   (at cable)
//


// clang-format off
#ifdef ESP32
  // Pin Setup per ESP32 (Gameboy Printer)
  //                  | Arduino Pin | Gameboy Link Pin  |
  #define GBP_VCC_PIN               // Pin 1            : White(5.0V) ->(Unused)
  #define GBP_SO_PIN        12      // Pin 2            : Green       -> GPIO12
  #define GBP_SI_PIN        13      // Pin 3            : Blue        -> GPIO13
  #define GBP_SD_PIN                // Pin 4            : Serial Data (Unused)
  #define GBP_SC_PIN        14      // Pin 5            : Orange      -> GPIO14 (deve essere un pin con interrupt)
  #define GBP_GND_PIN               // Pin 6            : Black       -> GND
  #define LED_STATUS_PIN    2       // LED interno (controlla la tua scheda!)
  
  // Pin per il Display TFT 1.8" (assicurarsi di non sovrapporre con quelli del GBP)
  // Pin per il display ST7735
  #define TFT_CS_PIN    5    // Chip Select del display
  #define TFT_DC_PIN    4    // Data/Command
  #define TFT_RST_PIN   15   // Reset del display

  #define TFT_LED_PIN   22   // Led Backlight display 

  // Pin per la scheda SD
  #define SD_CS_PIN     21   // Chip Select SD card Reader 

#elif defined(ESP8266)
  // Pin Setup per ESP8266 (Gameboy Printer)
  //                  | Arduino Pin | Gameboy Link Pin  |
  #define GBP_VCC_PIN               // Pin 1            : 5.0V (Unused)
  #define GBP_SO_PIN       13       // Pin 2            : ESP-pin 7 MOSI (Serial OUTPUT)
  #define GBP_SI_PIN       12       // Pin 3            : ESP-pin 6 MISO (Serial INPUT)
  #define GBP_SD_PIN                // Pin 4            : Serial Data  (Unused)
  #define GBP_SC_PIN       14       // Pin 5            : ESP-pin 5 CLK  (Serial Clock)
  #define GBP_GND_PIN               // Pin 6            : GND (Attach to GND Pin)
  #define LED_STATUS_PIN    2       // Internal LED blink on packet reception
  
  // Per l'ESP8266 l'hardware SPI usa i pin 12,13,14 (già usati dal GBP) -> uso Software SPI per il TFT
  #define TFT_CS_PIN    15
  #define TFT_DC_PIN    4
  #define TFT_RST_PIN   5
  #define TFT_SCLK_PIN  16
  #define TFT_MOSI_PIN  0

#else
  // Pin Setup per Arduinos (Gameboy Printer)
  //                  | Arduino Pin | Gameboy Link Pin  |
  #define GBP_VCC_PIN               // Pin 1            : 5.0V (Unused)
  #define GBP_SO_PIN        4       // Pin 2            : Serial OUTPUT
  #define GBP_SI_PIN        3       // Pin 3            : Serial INPUT
  #define GBP_SD_PIN                // Pin 4            : Serial Data  (Unused)
  #define GBP_SC_PIN        2       // Pin 5            : Serial Clock (Interrupt)
  #define GBP_GND_PIN               // Pin 6            : GND (Attach to GND Pin)
  #define LED_STATUS_PIN   13       // Internal LED blink on packet reception
  
  // Pin per il Display TFT 1.8" (evitare conflitti: 2,3,4 già usati, 13 è LED)
  #define TFT_CS_PIN    10
  #define TFT_DC_PIN    8
  #define TFT_RST_PIN   9

  /*Schema collegamento Schermo tft */
//  LED (Retroilluminazione)  ->	3.3V (o 5V se supportato)	Alimentazione per la retroilluminazione
//  SCK (Clock SPI)	          ->  D13 (SPI SCK)	Clock SPI (Serial Clock)
//  SDA (MOSI - Data SPI)	    ->  D11 (SPI MOSI)	Dati inviati ad alta velocità
//  A0 (DC - Data/Command)	  ->  D8	Differenzia dati da comandi
//  RESET	                    ->  D9	Reset hardware (necessario)
//  CS (Chip Select)	        ->  D10	Selezione del dispositivo SPI
//  GND	                      ->  GND	Massa (collegamento a terra)
//  VCC	                      ->  3.3V	⚠ Molti display sono a 3.3V! (Alcuni supportano 5V)

#endif

// Inizializzazione oggetto display TFT
#ifdef ESP8266
  // Costruttore per Software SPI
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_MOSI_PIN, TFT_SCLK_PIN, TFT_RST_PIN);
#else
  // Per ESP32 e Arduino si usa l'hardware SPI
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
#endif
// clang-format on

/*******************************************************************************
*******************************************************************************/

// Dev Note: Gamboy camera sends data payload of 640 bytes usually

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
#define GBP_BUFFER_SIZE 400
#else
#define GBP_BUFFER_SIZE 650
#endif

/* Serial IO */
// This circular buffer contains a stream of raw packets from the gameboy
uint8_t gbp_serialIO_raw_buffer[GBP_BUFFER_SIZE] = { 0 };

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
/* Packet Buffer */
gbp_pkt_t gbp_pktState                                 = { GBP_REC_NONE, 0 };
uint8_t gbp_pktbuff[GBP_PKT_PAYLOAD_BUFF_SIZE_IN_BYTE] = { 0 };
uint8_t gbp_pktbuffSize                                = 0;
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
gbp_pkt_tileAcc_t tileBuff = { 0 };
#endif
#endif

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
inline void gbp_packet_capture_loop();
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
inline void gbp_parse_packet_loop();
#endif

/*******************************************************************************
  Interrupt Service Routine
*******************************************************************************/

#ifdef ESP32
void IRAM_ATTR serialClock_ISR(void)
#elif defined(ESP8266)
void ICACHE_RAM_ATTR serialClock_ISR(void)
#else
void serialClock_ISR(void)
#endif
{
  // Serial Clock (1 = Rising Edge) (0 = Falling Edge); Master Output Slave Input (This device is slave)
#ifdef GBP_FEATURE_USING_RISING_CLOCK_ONLY_ISR
  const bool txBit = gpb_serial_io_OnRising_ISR(digitalRead(GBP_SO_PIN));
#else
  const bool txBit = gpb_serial_io_OnChange_ISR(digitalRead(GBP_SC_PIN), digitalRead(GBP_SO_PIN));
#endif
  digitalWrite(GBP_SI_PIN, txBit ? HIGH : LOW);
}

String outputBuffer = "";  // Buffer globale per salvare l'output


/*******************************************************************************
  Main Setup and Loop
*******************************************************************************/

void setup(void)
{
  // delay(1000);  // Ritardo per stabilizzare la seriale USB
  // Config Serial
  // Has to be fast or it will not transfer the image fast enough to the computer
  Serial.begin(115200);

  // Wait for Serial to be ready
  while (!Serial) { ; }

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
  }

  // Inizializzazione del display TFT
  Serial.println("Inizializzazione del display TFT");
  tft.initR(INITR_BLACKTAB);  // Inizializza il display (scegli il tab corretto per il tuo modello)
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setRotation(3);
  pinMode(TFT_LED_PIN, OUTPUT);
  digitalWrite(TFT_LED_PIN, HIGH);

  //Connect_to_printer();  //makes an attempt to switch in printer mode

  /* Pins from gameboy link cable */
  pinMode(GBP_SC_PIN, INPUT);
  pinMode(GBP_SO_PIN, INPUT);
  pinMode(GBP_SI_PIN, OUTPUT);

  /* Default link serial out pin state */
  digitalWrite(GBP_SI_PIN, LOW);

  /* LED Indicator */
  pinMode(LED_STATUS_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);

  /* Setup */
  gpb_serial_io_init(sizeof(gbp_serialIO_raw_buffer), gbp_serialIO_raw_buffer);

  /* Attach ISR */
#ifdef GBP_FEATURE_USING_RISING_CLOCK_ONLY_ISR
  attachInterrupt(digitalPinToInterrupt(GBP_SC_PIN), serialClock_ISR, RISING);  // attach interrupt handler
#else
  attachInterrupt(digitalPinToInterrupt(GBP_SC_PIN), serialClock_ISR, CHANGE);  // attach interrupt handler
#endif

  /* Packet Parser */
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  gbp_pkt_init(&gbp_pktState);
#endif

#define VERSION_STRING "V3.2.1 (Copyright (C) 2022 Brian Khuu)"

  /* Welcome Message */
#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
  Serial.println(F("// GAMEBOY PRINTER Packet Capture " VERSION_STRING));
  Serial.println(F("// Note: Each byte is from each GBP packet is from the gameboy"));
  Serial.println(F("//       except for the last two bytes which is from the printer"));
  Serial.println(F("// JS Raw Packet Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_raw_decoder.html"));
  tft.println(F("// GAMEBOY PRINTER Packet Capture " VERSION_STRING));
  tft.println(F("// Note: Each byte is from each GBP packet is from the gameboy"));
  tft.println(F("//       except for the last two bytes which is from the printer"));
  tft.println(F("// JS Raw Packet Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_raw_decoder.html"));
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  Serial.println(F("// GAMEBOY PRINTER Emulator " VERSION_STRING));
  Serial.println(F("// Note: Each hex encoded line is a gameboy tile"));
  Serial.println(F("// JS Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_decoder.html"));
  tft.println(F("// GAMEBOY PRINTER Emulator " VERSION_STRING));
  tft.println(F("// Note: Each hex encoded line is a gameboy tile"));
  tft.println(F("// JS Decoder: https://mofosyne.github.io/arduino-gameboy-printer-emulator/GameBoyPrinterDecoderJS/gameboy_printer_js_decoder.html"));
#endif
  Serial.println(F("// --- GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007 ---"));
  Serial.println(F("// This program comes with ABSOLUTELY NO WARRANTY;"));
  Serial.println(F("// This is free software, and you are welcome to redistribute it"));
  Serial.println(F("// under certain conditions. Refer to LICENSE file for detail."));
  Serial.println(F("// ---"));
  tft.println(F("// --- GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007 ---"));
  tft.println(F("// This program comes with ABSOLUTELY NO WARRANTY;"));
  tft.println(F("// This is free software, and you are welcome to redistribute it"));
  tft.println(F("// under certain conditions. Refer to LICENSE file for detail."));
  tft.println(F("// ---"));
  Serial.flush();

  delay(1000);

  drawBMP("/main_screen.bmp", 0, 0);
}  // setup()

void loop()
{
  static uint16_t sioWaterline = 0;

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
  gbp_packet_capture_loop();
#endif
#ifdef GBP_FEATURE_PARSE_PACKET_MODE
  gbp_parse_packet_loop();
#endif

  // Trigger Timeout and reset the printer if byte stopped being received.
  static uint32_t last_millis = 0;
  uint32_t curr_millis        = millis();
  if (curr_millis > last_millis)
  {
    uint32_t elapsed_ms = curr_millis - last_millis;
    if (gbp_serial_io_timeout_handler(elapsed_ms))
    {
      Serial.println("");
      Serial.print("// Completed ");
      Serial.print("(Memory Waterline: ");
      Serial.print(gbp_serial_io_dataBuff_waterline(false));
      Serial.print("B out of ");
      Serial.print(gbp_serial_io_dataBuff_max());
      Serial.println("B)");
      Serial.flush();
      //Serial.println("test outputBuffer:");
      //Serial.println(outputBuffer);
      digitalWrite(LED_STATUS_PIN, LOW);

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
      gbp_pkt_reset(&gbp_pktState);
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
      tileBuff.count = 0;
#endif
#endif

      convertOutputBufferToBmp(outputBuffer);
      outputBuffer = "";
    }
  }
  last_millis = curr_millis;

  // Diagnostics Console
  while (Serial.available() > 0)
  {
    switch (Serial.read())
    {
      case '?':
        Serial.println("d=debug, ?=help");
        break;

      case 'd':
        Serial.print("waterline: ");
        Serial.print(gbp_serial_io_dataBuff_waterline(false));
        Serial.print("B out of ");
        Serial.print(gbp_serial_io_dataBuff_max());
        Serial.println("B");
        break;
    }
  };
}  // loop()

/******************************************************************************/

#ifdef GBP_FEATURE_PARSE_PACKET_MODE
inline void gbp_parse_packet_loop(void)
{
  const char nibbleToCharLUT[] = "0123456789ABCDEF";
  for (int i = 0; i < gbp_serial_io_dataBuff_getByteCount(); i++)
  {
    if (gbp_pkt_processByte(&gbp_pktState, (const uint8_t)gbp_serial_io_dataBuff_getByte(), gbp_pktbuff, &gbp_pktbuffSize, sizeof(gbp_pktbuff)))
    {
      if (gbp_pktState.received == GBP_REC_GOT_PACKET)
      {
        digitalWrite(LED_STATUS_PIN, HIGH);
        Serial.print((char)'{');
        Serial.print("\"command\":\"");
        Serial.print(gbpCommand_toStr(gbp_pktState.command));
        Serial.print("\"");
        if (gbp_pktState.command == GBP_COMMAND_INQUIRY)
        {
          // !{"command":"INQY","status":{"lowbatt":0,"jam":0,"err":0,"pkterr":0,"unproc":1,"full":0,"bsy":0,"chk_err":0}}
          Serial.print(", \"status\":{");
          Serial.print("\"LowBat\":");
          Serial.print(gpb_status_bit_getbit_low_battery(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER2\":");
          Serial.print(gpb_status_bit_getbit_other_error(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER1\":");
          Serial.print(gpb_status_bit_getbit_paper_jam(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"ER0\":");
          Serial.print(gpb_status_bit_getbit_packet_error(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Untran\":");
          Serial.print(gpb_status_bit_getbit_unprocessed_data(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Full\":");
          Serial.print(gpb_status_bit_getbit_print_buffer_full(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Busy\":");
          Serial.print(gpb_status_bit_getbit_printer_busy(gbp_pktState.status) ? '1' : '0');
          Serial.print(",\"Sum\":");
          Serial.print(gpb_status_bit_getbit_checksum_error(gbp_pktState.status) ? '1' : '0');
          Serial.print((char)'}');
        }
        if (gbp_pktState.command == GBP_COMMAND_PRINT)
        {
          //!{"command":"PRNT","sheets":1,"margin_upper":1,"margin_lower":3,"pallet":228,"density":64 }
          Serial.print(", \"sheets\":");
          Serial.print(gbp_pkt_printInstruction_num_of_sheets(gbp_pktbuff));
          Serial.print(", \"margin_upper\":");
          Serial.print(gbp_pkt_printInstruction_num_of_linefeed_before_print(gbp_pktbuff));
          Serial.print(", \"margin_lower\":");
          Serial.print(gbp_pkt_printInstruction_num_of_linefeed_after_print(gbp_pktbuff));
          Serial.print(", \"pallet\":");
          Serial.print(gbp_pkt_printInstruction_palette_value(gbp_pktbuff));
          Serial.print(", \"density\":");
          Serial.print(gbp_pkt_printInstruction_print_density(gbp_pktbuff));
        }
        if (gbp_pktState.command == GBP_COMMAND_DATA)
        {
          //!{"command":"DATA", "compressed":0, "more":0}
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
          Serial.print(", \"compressed\":0");  // Already decompressed by us, so no need to do so
#else
          Serial.print(", \"compressed\":");
          Serial.print(gbp_pktState.compression);
#endif
          Serial.print(", \"more\":");
          Serial.print((gbp_pktState.dataLength != 0) ? '1' : '0');
        }
        Serial.println((char)'}');
        Serial.flush();
      }
      else
      {
#ifdef GBP_FEATURE_PARSE_PACKET_USE_DECOMPRESSOR
        // Required for more complex games with compression support
        while (gbp_pkt_decompressor(&gbp_pktState, gbp_pktbuff, gbp_pktbuffSize, &tileBuff))
        {
          if (gbp_pkt_tileAccu_tileReadyCheck(&tileBuff))
          {
            // Got Tile
            for (int i = 0; i < GBP_TILE_SIZE_IN_BYTE; i++)
            {
              const uint8_t data_8bit = tileBuff.tile[i];
              if (i == GBP_TILE_SIZE_IN_BYTE - 1)
              {
                char hex[3];
                hex[0] = nibbleToCharLUT[(data_8bit >> 4) & 0xF];
                hex[1] = nibbleToCharLUT[(data_8bit >> 0) & 0xF];
                hex[2] = '\0';
                myPrint(hex);
              }
              else
              {
                char hex[4];
                hex[0] = nibbleToCharLUT[(data_8bit >> 4) & 0xF];
                hex[1] = nibbleToCharLUT[(data_8bit >> 0) & 0xF];
                hex[2] = ' ';
                hex[3] = '\0';
                myPrint(hex);
              }
            }
            Serial.flush();
          }
        }
#else
        // Simplified support for gameboy camera only application
        // Dev Note: Good for checking if everything above decompressor is working
        if (gbp_pktbuffSize > 0)
        {
          // Got Tile
          for (int i = 0; i < gbp_pktbuffSize; i++)
          {
            const uint8_t data_8bit = gbp_pktbuff[i];
            if (i == gbp_pktbuffSize - 1)
            {
              char hex[3];
              hex[0] = nibbleToCharLUT[(data_8bit >> 4) & 0xF];
              hex[1] = nibbleToCharLUT[(data_8bit >> 0) & 0xF];
              hex[2] = '\0';
              myPrint(hex);
            }
            else
            {
              char hex[4];
              hex[0] = nibbleToCharLUT[(data_8bit >> 4) & 0xF];
              hex[1] = nibbleToCharLUT[(data_8bit >> 0) & 0xF];
              hex[2] = ' ';
              hex[3] = '\0';
              myPrint(hex);
            }
          }
          Serial.flush();
        }
#endif
      }
    }
  }
}
#endif

#ifdef GBP_FEATURE_PACKET_CAPTURE_MODE
inline void gbp_packet_capture_loop()
{
  /* tiles received */
  static uint32_t byteTotal     = 0;
  static uint32_t pktTotalCount = 0;
  static uint32_t pktByteIndex  = 0;
  static uint16_t pktDataLength = 0;
  const size_t dataBuffCount    = gbp_serial_io_dataBuff_getByteCount();
  if (((pktByteIndex != 0) && (dataBuffCount > 0)) || ((pktByteIndex == 0) && (dataBuffCount >= 6)))
  {
    const char nibbleToCharLUT[] = "0123456789ABCDEF";
    uint8_t data_8bit = 0;
    for (int i = 0; i < dataBuffCount; i++)
    {  
      // Inizio di un nuovo pacchetto
      if (pktByteIndex == 0)
      {
        pktDataLength = gbp_serial_io_dataBuff_getByte_Peek(4);
        pktDataLength |= (gbp_serial_io_dataBuff_getByte_Peek(5) << 8) & 0xFF00;
        digitalWrite(LED_STATUS_PIN, HIGH);
      }
      // Lettura del byte e stampa in esadecimale
      data_8bit = gbp_serial_io_dataBuff_getByte();
      
      // Stampa il nibble alto
      myPrint((char)nibbleToCharLUT[(data_8bit >> 4) & 0xF]);
      // Stampa il nibble basso
      myPrint((char)nibbleToCharLUT[(data_8bit >> 0) & 0xF]);
      
      // Controllo per suddividere i pacchetti
      if ((pktByteIndex > 5) && (pktByteIndex >= (9 + pktDataLength)))
      {
        digitalWrite(LED_STATUS_PIN, LOW);
        myPrintln("");  // Aggiunge il ritorno a capo
        pktByteIndex = 0;
        pktTotalCount++;
      }
      else
      {
        myPrint(' ');  // Stampa uno spazio
        pktByteIndex++;  // Incrementa il contatore del byte nel pacchetto
        byteTotal++;     // Incrementa il totale dei byte
      }
    }
    Serial.flush();
  }
}
#endif

void Connect_to_printer()
{
#if GAME_BOY_PRINTER_MODE  //Printer mode
  pinMode(GBP_SC_PIN, OUTPUT);
  pinMode(GBP_SO_PIN, INPUT_PULLUP);
  pinMode(GBP_SI_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  const char INIT[] = { 0x88, 0x33, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };  //INIT command
  uint8_t junk, status;
  for (uint8_t i = 0; i < 10; i++)
  {
    junk = (printing(INIT[i]));
    if (i == 8)
    {
      status = junk;
    }
  }
  if (status == 0X81)  //Printer connected !
  {
    digitalWrite(GBP_SC_PIN, HIGH);  //acts like a real Game Boy
    digitalWrite(GBP_SI_PIN, LOW);   //acts like a real Game Boy
    Serial.println(F("// A printer is connected to the serial cable !!!"));
    Serial.println(F("// GAME BOY PRINTER I/O INTERFACE Made By Raphaël BOICHOT, 2023"));
    Serial.println(F("// Use with the GBCamera-Android-Manager: https://github.com/Mraulio/GBCamera-Android-Manager"));
    Serial.println(F("// Or with the PC-to-Game-Boy-Printer-interface: https://github.com/Raphael-Boichot/PC-to-Game-Boy-Printer-interface"));
    delay(100);
    Serial.begin(9600);
    while (!Serial) { ; }
    while (Serial.available() > 0)
    {  //flush the buffer from any remaining data
      Serial.read();
    }
    digitalWrite(LED_STATUS_PIN, HIGH);  //LED ON = PRINTER INTERFACE mode
    while (true)
    {
      if (Serial.available() > 0)
      {
        Serial.write(printing(Serial.read()));
      }
    }
  }
#endif
}

#if GAME_BOY_PRINTER_MODE      //Printer mode
char printing(char byte_sent)  // this function prints bytes to the serial
{
  bool bit_sent, bit_read;
  char byte_read;
  for (int i = 0; i <= 7; i++)
  {
    bit_sent = bitRead(byte_sent, 7 - i);
    digitalWrite(GBP_SC_PIN, LOW);
    digitalWrite(GBP_SI_PIN, bit_sent);  //GBP_SI_PIN is SOUT for the printer
    digitalWrite(LED_STATUS_PIN, bit_sent);
    delayMicroseconds(30);  //double speed mode
    digitalWrite(GBP_SC_PIN, HIGH);
    bit_read = (digitalRead(GBP_SO_PIN));  //GBP_SO_PIN is SIN for the printer
    bitWrite(byte_read, 7 - i, bit_read);
    delayMicroseconds(30);  //double speed mode
  }
  delayMicroseconds(0);  //optionnal delay between bytes, may be less than 1490 µs
  //  Serial.println(byte_sent, HEX);
  //  Serial.println(byte_read, HEX);
  return byte_read;
}
#endif

void myPrint(const char *s) {
  Serial.print(s);
  outputBuffer += s;  // Aggiunge la stringa al buffer
}

void myPrintln(const char *s) {
  Serial.println(s);
  outputBuffer += s;   // Aggiunge la stringa al buffer
  outputBuffer += "\n";  // Inserisce un ritorno a capo
}

void myPrint(char c) {
  Serial.print(c);
  outputBuffer += c;
}

void myPrintln(char c) {
  Serial.println(c);
  outputBuffer += c;
  outputBuffer += "\n";
}

// Funzioni di utilità per la lettura in little-endian
uint16_t read16(File &f) {
  uint16_t result;
  result  = f.read();         // byte meno significativo
  result |= f.read() << 8;     // byte più significativo
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  result  = f.read();         // byte meno significativo
  result |= f.read() << 8;
  result |= f.read() << 16;
  result |= f.read() << 24;   // byte più significativo
  return result;
}

// Funzione per disegnare un BMP da SPIFFS sul display
void drawBMP(const char *filename, int x, int y) {
  // Apri il file dal file system SPIFFS
  File bmpFile = SPIFFS.open(filename, "r");
  if (!bmpFile) {
    Serial.println("File BMP non trovato!");
    return;
  }
  
  // Controlla l'intestazione BMP: i primi due byte devono essere 'B' 'M'
  if (read16(bmpFile) != 0x4D42) {  // 0x4D42 = 'BM'
    Serial.println("Non è un file BMP!");
    bmpFile.close();
    return;
  }
  
  // Salta: dimensione del file e due campi riservati
  uint32_t fileSize      = read32(bmpFile);
  read32(bmpFile);  // campo riservato
  // Ottieni l'offset dove inizia il pixel array
  uint32_t bmpImageOffset = read32(bmpFile);
  
  // Leggi l'intestazione DIB (BITMAPINFOHEADER)
  uint32_t headerSize = read32(bmpFile);
  int32_t bmpWidth    = read32(bmpFile);
  int32_t bmpHeight   = read32(bmpFile);
  uint16_t planes     = read16(bmpFile);
  uint16_t bitDepth   = read16(bmpFile);
  uint32_t compression = read32(bmpFile);
  
  if (compression != 0 || bitDepth != 24) {
    Serial.println("File BMP non supportato! (richiesto: 24 bit non compresso)");
    bmpFile.close();
    return;
  }
  
  // Spostati all'inizio dei dati immagine
  bmpFile.seek(bmpImageOffset);
  
  // Calcola la dimensione di ogni riga (le righe sono allineate a 4 byte)
  uint32_t rowSize = (bmpWidth * 3 + 3) & ~3;
  
  // Determina se l'immagine è "flip" (tipica per BMP, ovvero bottom-up)
  bool flip = true;
  if (bmpHeight < 0) {
    bmpHeight = -bmpHeight;
    flip = false;
  }
  
  // Itera su ciascuna riga dell'immagine
  for (int row = 0; row < bmpHeight; row++) {
    // Calcola la posizione di partenza per questa riga all'interno del file
    uint32_t pos;
    if (flip) {
      pos = bmpImageOffset + (bmpHeight - 1 - row) * rowSize;
    } else {
      pos = bmpImageOffset + row * rowSize;
    }
    bmpFile.seek(pos);
    
    // Itera su ciascun pixel della riga
    for (int col = 0; col < bmpWidth; col++) {
      // Leggi i 3 byte del pixel: BMP usa l'ordine B, G, R
      uint8_t b = bmpFile.read();
      uint8_t g = bmpFile.read();
      uint8_t r = bmpFile.read();
      
      // Converti il colore in formato 16-bit 565 utilizzando il metodo della libreria
      uint16_t color = tft.color565(r, g, b);
      
      // Disegna il pixel nel punto (x+col, y+row)
      tft.drawPixel(x + col, y + row, color);
    }
  }
  
  bmpFile.close();
  Serial.println("BMP disegnato!");
}
