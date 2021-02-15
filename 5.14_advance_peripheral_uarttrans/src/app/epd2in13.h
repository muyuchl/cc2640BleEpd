
#ifndef EPD_2IN13_H
#define EPD_2IN13_H

#include <stdint.h>

// Display resolution
#define EPD_2IN13_WIDTH       104
#define EPD_2IN13_HEIGHT      212

// #define EPD_2IN13_FULL			0
// #define EPD_2IN13_PART			1

void epd_hw_init();

void EPD_2IN13_Init();
void EPD_2IN13_Clear(void);
void EPD_2IN13_Display(const uint8_t *Image);

void EPD_2IN13_PrepareBlkRAM(void);
void EPD_2IN13_PrepareRedRAM(void);
void EPD_2IN13_WriteRAM(const uint8_t *buf, const int len);

void EPD_2IN13_UpdateDisplay(void);

// void EPD_2IN13_DisplayPart(UBYTE *Image);
// void EPD_2IN13_DisplayPartBaseImage(UBYTE *Image);

void EPD_2IN13_Sleep(void);

#endif
