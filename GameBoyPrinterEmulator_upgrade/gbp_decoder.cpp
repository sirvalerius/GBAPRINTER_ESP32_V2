/*
 GbpDecoder.cpp
 ---------------
 Implementazione della libreria per la decodifica dei pacchetti e conversione dello stream esadecimale in immagine BMP.
 
 Questa libreria integra:
   - La funzione convertOutputBufferToBmp() che analizza la stringa contenuta in outputBuffer,
     converte coppie di caratteri esadecimali in byte e li invia al decoder.
   - La funzione gbpdecoder_gotByte() che processa ciascun byte tramite la pipeline di decodifica,
     ricostruendo le tiles e, in caso di comando PRINT, scrivendo il file BMP (o mostrando una preview).
   - Una funzione initGbpDecoder() per inizializzare le variabili e la logica di decodifica.
 
 Dipendenze:
   - gameboy_printer_protocol.h
   - gbp_pkt.h
   - gbp_tiles.h
   - gbp_bmp.h
   - Arduino.h, SPIFFS.h, stdio.h, stdlib.h
*/

#include "GbpDecoder.h"

#include <Arduino.h>
#include <SPIFFS.h>
#include <stdio.h>
#include <stdlib.h>

// Includi i moduli del progetto
#include "gameboy_printer_protocol.h"
#include "gbp_pkt.h"
#include "gbp_tiles.h"
#include "gbp_bmp.h"

// Variabili interne al decoder
static bool verbose_flag = false;   // Impostabile tramite initGbpDecoder()
static bool display_flag = false;   // Se true, mostra una preview via VT100

static uint8_t pktCounter = 0;
static gbp_pkt_t gbp_pktBuff = { GBP_REC_NONE, 0 };
static uint8_t gbp_pktbuff[GBP_PKT_PAYLOAD_BUFF_SIZE_IN_BYTE] = {0};
static uint8_t gbp_pktbuffSize = 0;
static gbp_pkt_tileAcc_t tileBuff = {0};
static gbp_tile_t gbp_tiles = {0};
static gbp_bmp_t gbp_bmp = {0};

// Variabili di configurazione
// Definisce il pallet di colori (default: bianco, grigio chiaro, grigio scuro, nero)
uint32_t palletColor[4] = { 0xFFFFFF, 0xAAAAAA, 0x555555, 0x000000 };

// Nome del file di output BMP (modificabile se necessario)
char ofilenameBuf[255] = "gbpOut.bmp";


/**************************************************************************
 * Funzione: convertOutputBufferToBmp
 * ------------------------------------------------
 * Scorre la stringa outputBuffer (contenente lo stream in formato esadecimale),
 * converte ogni coppia di nibble in un byte e chiama gbpdecoder_gotByte() per processarlo.
 **************************************************************************/
void convertOutputBufferToBmp(const String &outputBuffer) {
  int len = outputBuffer.length();
  bool lowNibFound = false;
  uint8_t currentByte = 0;
  
  for (int i = 0; i < len; i++) {
    char ch = outputBuffer.charAt(i);
    
    // Salta spazi, ritorni a capo e altri caratteri inutili
    if (ch == ' ' || ch == '\n' || ch == '\r')
      continue;
      
    int nibble = -1;
    if (ch >= '0' && ch <= '9')
      nibble = ch - '0';
    else if (ch >= 'A' && ch <= 'F')
      nibble = ch - 'A' + 10;
    else if (ch >= 'a' && ch <= 'f')
      nibble = ch - 'a' + 10;
      
    if (nibble == -1)
      continue;
      
    if (!lowNibFound) {
      currentByte = nibble << 4;
      lowNibFound = true;
    } else {
      currentByte |= nibble;
      lowNibFound = false;
      // Invio il byte ottenuto alla pipeline di decodifica
      gbpdecoder_gotByte(currentByte);
    }
  }
}


/**************************************************************************
 * Funzione: gbpCommand_toStr
 * ------------------------------------------------
 * Converte un comando numerico in una stringa leggibile.
 **************************************************************************/
const char * gbpCommand_toStr(int val) {
  switch (val) {
    case GBP_COMMAND_INIT:    return "INIT";
    case GBP_COMMAND_PRINT:   return "PRNT";
    case GBP_COMMAND_DATA:    return "DATA";
    case GBP_COMMAND_BREAK:   return "BREK";
    case GBP_COMMAND_INQUIRY: return "INQY";
    default:                  return "?";
  }
}


/**************************************************************************
 * Funzione: gbpdecoder_gotByte
 * ------------------------------------------------
 * Processa ciascun byte ricevuto, aggiornando la pipeline di decodifica.
 * Se viene rilevato un pacchetto completo, in caso di comando PRINT:
 *    - Se display_flag è attivo, mostra una preview via VT100.
 *    - Altrimenti, gestisce la scrittura dello stream BMP in streaming.
 **************************************************************************/
void gbpdecoder_gotByte(const uint8_t byte)
{
  if (gbp_pkt_processByte(&gbp_pktBuff, byte, gbp_pktbuff, &gbp_pktbuffSize, sizeof(gbp_pktbuff))) {
    
    if (gbp_pktBuff.received == GBP_REC_GOT_PACKET) {
      
      pktCounter++;
      
      if (verbose_flag) {
        // Su Arduino, l'uso di printf potrebbe richiedere una libreria extra o essere sostituito con Serial.print().
        // Qui manteniamo l'approccio desktop, modificabile in base alle necessità.
        printf("// %s | compression: %1u, dlength: %3u, printerID: 0x%02X, status: %u | %d | ",
               gbpCommand_toStr(gbp_pktBuff.command),
               (unsigned) gbp_pktBuff.compression,
               (unsigned) gbp_pktBuff.dataLength,
               (unsigned) gbp_pktBuff.printerID,
               (unsigned) gbp_pktBuff.status,
               (unsigned) pktCounter);
        for (int i = 0; i < gbp_pktbuffSize; i++) {
          printf("%02X ", gbp_pktbuff[i]);
        }
        printf("\r\n");
      }
      
      if (gbp_pktBuff.command == GBP_COMMAND_PRINT) {
        // Determina se il comando prevede la "corte carta"
        const bool cutPaper = ((gbp_pktbuff[GBP_PRINT_INSTRUCT_INDEX_NUM_OF_LINEFEED] & 0xF) != 0);
        
        // Processa le tiles per la stampa
        gbp_tiles_print(&gbp_tiles,
                        gbp_pktbuff[GBP_PRINT_INSTRUCT_INDEX_NUM_OF_SHEETS],
                        gbp_pktbuff[GBP_PRINT_INSTRUCT_INDEX_NUM_OF_LINEFEED],
                        gbp_pktbuff[GBP_PRINT_INSTRUCT_INDEX_PALETTE_VALUE],
                        gbp_pktbuff[GBP_PRINT_INSTRUCT_INDEX_PRINT_DENSITY]);
        
        if (display_flag) {
          // Preview via VT100 (utilizzo di escape codes ANSI per colorare lo sfondo)
          for (int j = 0; j < (GBP_TILE_PIXEL_HEIGHT * gbp_tiles.tileRowOffset); j++) {
            for (int i = 0; i < (GBP_TILE_PIXEL_WIDTH * GBP_TILES_PER_LINE); i++) {
              const int pixel = 0b11 & (gbp_tiles.bmpLineBuffer[j][GBP_TILE_2BIT_LINEPACK_INDEX(i)]
                                        >> GBP_TILE_2BIT_LINEPACK_BITOFFSET(i));
              int b = 0;
              switch (pixel) {
                default:
                case 3: b = 0; break;
                case 2: b = 64; break;
                case 1: b = 130; break;
                case 0: b = 255; break;
              }
              printf("\x1B[48;2;%d;%d;%dm \x1B[0m", b, b, b);
            }
            printf("\r\n");
          }
          gbp_tiles_reset(&gbp_tiles);
        } else {
          // Scrittura BMP in streaming
          if (!gbp_bmp_isopen(&gbp_bmp)) {
            gbp_bmp_open(&gbp_bmp, ofilenameBuf, GBP_TILE_PIXEL_WIDTH * GBP_TILES_PER_LINE);
          }
          for (int j = 0; j < gbp_tiles.tileRowOffset; j++) {
            const long int tileHeightIncrement = GBP_TILE_PIXEL_HEIGHT * GBP_BMP_MAX_TILE_HEIGHT;
            gbp_bmp_add(&gbp_bmp,
                        (const uint8_t *)&gbp_tiles.bmpLineBuffer[tileHeightIncrement * j][0],
                        (GBP_TILE_PIXEL_WIDTH * GBP_TILES_PER_LINE),
                        tileHeightIncrement,
                        palletColor);
          }
          gbp_tiles_reset(&gbp_tiles);
          if (cutPaper) {
            gbp_bmp_render(&gbp_bmp);
          }
        }
      }
    } else {
      // Support per payload compressi
      while (gbp_pkt_decompressor(&gbp_pktBuff, gbp_pktbuff, gbp_pktbuffSize, &tileBuff)) {
        if (gbp_pkt_tileAccu_tileReadyCheck(&tileBuff)) {
          if (gbp_tiles_line_decoder(&gbp_tiles, tileBuff.tile)) {
            // La linea decodificata è pronta (eventuali operazioni di debug o anteprima possono essere inserite qui)
          }
        }
      }
    }
  }
}


/**************************************************************************
 * Funzione: initGbpDecoder
 * ------------------------------------------------
 * Inizializza le variabili e la logica del decoder.
 **************************************************************************/
void initGbpDecoder(bool verbose, bool display) {
  verbose_flag = verbose;
  display_flag = display;
  pktCounter = 0;
  gbp_pkt_init(&gbp_pktBuff);
  gbp_pktbuffSize = 0;
  memset(gbp_pktbuff, 0, sizeof(gbp_pktbuff));
  memset(&tileBuff, 0, sizeof(tileBuff));
  gbp_tiles_reset(&gbp_tiles);
  // Se esiste una funzione di inizializzazione per il modulo BMP, invocala qui
  // ad esempio: gbp_bmp_init(&gbp_bmp);
}
