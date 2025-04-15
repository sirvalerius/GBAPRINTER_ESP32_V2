#ifndef GBP_DECODER_H
#define GBP_DECODER_H

#include <Arduino.h>

// Inizializza la logica di decodifica (es. eventualmente gbp_pkt_init, ecc.)
void initGbpDecoder();

// Funzione che converte il buffer (stringa esadecimale) in byte e li manda al decoder
void convertOutputBufferToBmp(const String &outputBuffer);

// La funzione gbpdecoder_gotByte deve essere integrata o collegata al sistema di decodifica
void gbpdecoder_gotByte(const uint8_t byte);

#endif