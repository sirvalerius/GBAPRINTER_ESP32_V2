#ifndef GBP_DECODER_H
#define GBP_DECODER_H

#include <Arduino.h>

// Inizializza la logica di decodifica e imposta verbose/display
void initGbpDecoder(bool verbose, bool display);

// Converte la stringa esadecimale in byte e li passa al decoder
void convertOutputBufferToBmp(const String &outputBuffer);

// Riceve un singolo byte e lo processa
void gbpdecoder_gotByte(const uint8_t byte);

#endif // GBP_DECODER_H