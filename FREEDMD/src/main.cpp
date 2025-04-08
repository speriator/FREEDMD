/*
 * main.cpp - Stein-Lab.com project under GPLv3 License
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * (C) Stein-Lab.com
 */

#include "Arduino.h"
#include "stm32f4xx.h"
#include <string.h>

HardwareSerial SerialDebug(PB7, PB6);

// ----- DMD Input Pin Definitions -----
#define DMD_DOTCLK         GPIO_PIN_3    // PB3
#define DMD_DATA           GPIO_PIN_4    // PB4
#define DMD_COLUMN_LATCH   GPIO_PIN_8    // PB8
#define DMD_ROW_CLOCK      GPIO_PIN_9    // PB9
#define DMD_ROW_DATA       GPIO_PIN_12   // PB12
#define DMD_DISPLAY_ENABLE GPIO_PIN_1    // PB1

// ----- HUB75 Output Pin Definitions -----
#define DISPLAY_R0        GPIO_PIN_0    // PE0
#define DISPLAY_G0        GPIO_PIN_1    // PE1
#define DISPLAY_B0        GPIO_PIN_2    // PE2
#define DISPLAY_R1        GPIO_PIN_3    // PE3
#define DISPLAY_G1        GPIO_PIN_4    // PE4
#define DISPLAY_B1        GPIO_PIN_5    // PE5
#define DISPLAY_A         GPIO_PIN_10   // PD10
#define DISPLAY_B         GPIO_PIN_11   // PD11
#define DISPLAY_C         GPIO_PIN_12   // PD12
#define DISPLAY_D         GPIO_PIN_13   // PD13
#define DISPLAY_CLK       GPIO_PIN_6    // PE6
#define DISPLAY_LATCH     GPIO_PIN_15   // PD15
#define DISPLAY_OE        GPIO_PIN_7    // PC7

// ----- Display Configuration ----- 
#define NUM_ROWS 32
#define NUM_COLS 128
#define FRAME_LINES 32        // Number of lines per frame (e.g., 32 lines)
#define NUM_FRAME_BUFFERS 3   // Triple buffering
#define WORD_BUFFER_SIZE 8    // 8 words of 16 bits = 128 bits per line

// ----- Global Buffers and Variables ----- 
volatile uint16_t spi16Buffer[WORD_BUFFER_SIZE];  // SPI reception buffer (16-bit words)
volatile uint8_t  spi16BufferIndex = 0;             // Current index in the 16-bit buffer

// Triple buffering frame memory; each line is 128 bits (8 words, 16 bits each)
volatile uint16_t frameBuffer[NUM_FRAME_BUFFERS][FRAME_LINES][WORD_BUFFER_SIZE];
volatile uint8_t spiByteIndex = 0;   // Unused with direct transfer
volatile uint16_t lineCounter = 0;   // Current line number in the active frame
volatile uint32_t frameCount = 0;    // Count of complete frames

// Buffer management indexes
volatile uint8_t currentFrameIndex = 0; // Active buffer index for filling
volatile uint8_t readyFrameIndex = 0;   // Buffer index ready for refresh
volatile uint8_t newFrameReady = 0;     // Flag indicating a complete frame is ready

volatile uint8_t strobe = false;        // Column strobe flag (for debugging/monitoring)

// ----- System Clock Configuration (using HAL) -----
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;  
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while(1);
  }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // SYSCLK = 168 MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;          
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;          

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    while(1);
  }
}

// ----- SPI1 Initialization in Bidirectional Slave Mode (16-bit) -----
void SPI1_Slave_Bidirectional_Init(void) {
  // Enable clocks for SPI1 and GPIOB
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  
  // Configure PB3 and PB4 to Alternate Function (AF5) for SPI1
  GPIOB->MODER &= ~((3UL << (3 * 2)) | (3UL << (4 * 2)));
  GPIOB->MODER |= ((2UL << (3 * 2)) | (2UL << (4 * 2)));
  
  // Select alternate function AF5 for SPI1 on PB3 and PB4
  GPIOB->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (4 * 4)));
  GPIOB->AFR[0] |= ((5UL << (3 * 4)) | (5UL << (4 * 4)));
  
  // Disable SPI1 to configure registers
  SPI1->CR1 &= ~SPI_CR1_SPE;
  
  // Setup SPI in bidirectional mode (BIDIMODE enabled, BIDIOE disabled for receive only)
  SPI1->CR1 = SPI_CR1_BIDIMODE;
  SPI1->CR1 &= ~SPI_CR1_BIDIOE;
  
  // Ensure slave mode by clearing MSTR
  SPI1->CR1 &= ~SPI_CR1_MSTR;
  
  // Set CPOL=0 and CPHA=0
  SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
  
  // Enable 16-bit data frame mode
  SPI1->CR1 |= SPI_CR1_DFF;
  
  // Re-enable SPI1 and enable RXNE interrupt
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR2 |= SPI_CR2_RXNEIE;
  NVIC_EnableIRQ(SPI1_IRQn);
}

// ----- SPI1 Interrupt Handler (Data Reception in 16-bit Mode) -----
extern "C" void SPI1_IRQHandler(void) {
  if (SPI1->SR & SPI_SR_RXNE) {
    // Read 16-bit data from SPI1->DR (RXNE flag clears automatically)
    uint16_t data = *((volatile uint16_t *)&SPI1->DR);
    if (spi16BufferIndex < WORD_BUFFER_SIZE) {
      // Write data directly into the current frame and line
      frameBuffer[currentFrameIndex][lineCounter][spi16BufferIndex] = data;
      spi16BufferIndex++;
    }
    if(spi16BufferIndex >= WORD_BUFFER_SIZE) {
      // Disable SPI reception
      SPI1->CR1 &= ~SPI_CR1_SPE;
    }
    // End-of-line indication is handled in the column strobe ISR
  }
}

// ----- Column Latch (Strobe) Interrupt Handler (PB8) -----
// This ISR is triggered when PB8 (column latch) goes high, indicating the end of a 128-bit line.
extern "C" void columnStrobeISR(void) {
  // Reset the 16-bit word index for the next line
  spi16BufferIndex = 0;
  
  // Check DMD_ROW_DATA (PB12) to determine if it is the start of a new frame
  if (GPIOB->IDR & DMD_ROW_DATA) {
    // End of frame detected: current line completes the frame
    newFrameReady = 1;
    readyFrameIndex = currentFrameIndex;
    currentFrameIndex = (currentFrameIndex + 1) % NUM_FRAME_BUFFERS;
    lineCounter = 0;
    frameCount++;
  } else {
    // Continue with subsequent lines in the current frame
    if (lineCounter >= FRAME_LINES) {
      lineCounter = FRAME_LINES - 1;
    } else {
      lineCounter++;
    }
  }
  SPI1->DR;
  // Re-enable SPI reception
  SPI1->CR1 |= SPI_CR1_SPE;
  // Set strobe flag for debugging or further processing
  strobe = true;
}

// ----- DMD Pins Initialization -----
// Sets DMD pins as inputs with pull-down resistors
void DMD_InputPins_Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  
  // Set pins as input (MODER = 00) for DMD signals
  GPIOB->MODER &= ~(
      (0x3 << (3 * 2))  |  // PB3: DMD_DOTCLK
      (0x3 << (4 * 2))  |  // PB4: DMD_DATA
      (0x3 << (8 * 2))  |  // PB8: DMD_COLUMN_LATCH
      (0x3 << (9 * 2))  |  // PB9: DMD_ROW_CLOCK
      (0x3 << (12 * 2)) |  // PB12: DMD_ROW_DATA
      (0x3 << (1 * 2))     // PB1: DMD_DISPLAY_ENABLE
  );
  
  // Clear current pull-up/pull-down configurations
  GPIOB->PUPDR &= ~(
      (0x3 << (3 * 2))  |
      (0x3 << (4 * 2))  |
      (0x3 << (8 * 2))  |
      (0x3 << (9 * 2))  |
      (0x3 << (12 * 2)) |
      (0x3 << (1 * 2))
  );
  
  // Configure pins with pull-down resistors (10)
  GPIOB->PUPDR |= (
      (0x2 << (3 * 2))  |  // PB3: DMD_DOTCLK
      (0x2 << (4 * 2))  |  // PB4: DMD_DATA
      (0x2 << (8 * 2))  |  // PB8: DMD_COLUMN_LATCH
      (0x2 << (9 * 2))  |  // PB9: DMD_ROW_CLOCK
      (0x2 << (12 * 2)) |  // PB12: DMD_ROW_DATA
      (0x2 << (1 * 2))     // PB1: DMD_DISPLAY_ENABLE
  );
}

// ----- HUB75 Outputs Initialization ----- 
void initHUB75Outputs(void) {
  // Enable clocks for GPIOE (DISPLAY_R0, G0, B0, R1, G1, B1, CLK),
  // GPIOD (DISPLAY_A, B, C, D, LATCH) and GPIOC (DISPLAY_OE)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOCEN;
  
  // Configure Port E pins (PE0 to PE6) as outputs
  GPIOE->MODER &= ~((0x3 << (0 * 2)) | (0x3 << (1 * 2)) | (0x3 << (2 * 2)) |
                    (0x3 << (3 * 2)) | (0x3 << (4 * 2)) | (0x3 << (5 * 2)) | (0x3 << (6 * 2)));
  GPIOE->MODER |=  ((0x1 << (0 * 2)) | (0x1 << (1 * 2)) | (0x1 << (2 * 2)) |
                    (0x1 << (3 * 2)) | (0x1 << (4 * 2)) | (0x1 << (5 * 2)) | (0x1 << (6 * 2)));
  GPIOE->OSPEEDR |= ((0x3 << (0 * 2)) | (0x3 << (1 * 2)) | (0x3 << (2 * 2)) |
                     (0x3 << (3 * 2)) | (0x3 << (4 * 2)) | (0x3 << (5 * 2)) | (0x3 << (6 * 2)));
  
  // Configure Port D for DISPLAY_A, B, C, D, LATCH
  GPIOD->MODER &= ~((0x3 << (10 * 2)) | (0x3 << (11 * 2)) | (0x3 << (12 * 2)) |
                    (0x3 << (13 * 2)) | (0x3 << (15 * 2)));
  GPIOD->MODER |=  ((0x1 << (10 * 2)) | (0x1 << (11 * 2)) | (0x1 << (12 * 2)) |
                    (0x1 << (13 * 2)) | (0x1 << (15 * 2)));
  GPIOD->OSPEEDR |= ((0x3 << (10 * 2)) | (0x3 << (11 * 2)) | (0x3 << (12 * 2)) |
                     (0x3 << (13 * 2)) | (0x3 << (15 * 2)));
  
  // Configure Port C for DISPLAY_OE (PC7)
  GPIOC->MODER &= ~(0x3 << (7 * 2));
  GPIOC->MODER |=  (0x1 << (7 * 2));
  GPIOC->OSPEEDR |= (0x3 << (7 * 2));
  GPIOC->PUPDR &= ~(0x3 << (7 * 2));
}

// ----- Write Pixel Data for HUB75 -----
// Writes pixel data for top (rows 0-15) and bottom (rows 16-31)
void writePixelData(uint8_t topPixel, uint8_t bottomPixel) {
  if (topPixel) {
    GPIOE->BSRR = (DISPLAY_R0 | DISPLAY_G0 | DISPLAY_B0);
  } else {
    GPIOE->BSRR = ((DISPLAY_R0 | DISPLAY_G0 | DISPLAY_B0) << 16);
  }
  
  if (bottomPixel) {
    GPIOE->BSRR = (DISPLAY_R1 | DISPLAY_G1 | DISPLAY_B1);
  } else {
    GPIOE->BSRR = ((DISPLAY_R1 | DISPLAY_G1 | DISPLAY_B1) << 16);
  }
}

// ----- Toggle Display Clock -----
// Generates a clock pulse on DISPLAY_CLK (PE6)
void toggleDisplayClock() {
  GPIOE->BSRR = DISPLAY_CLK;           // Set clock high
  for(volatile int i = 0; i < 10; i++);  // Short delay
  GPIOE->BSRR = (DISPLAY_CLK << 16);     // Set clock low
}

// ----- Toggle Display Latch -----
// Latches the line data by pulsing DISPLAY_LATCH (PD15)
void toggleDisplayLatch() {
  GPIOD->BSRR = DISPLAY_LATCH;
  for(volatile int i = 0; i < 10; i++);
  GPIOD->BSRR = (DISPLAY_LATCH << 16);
}

// ----- Display Output Control -----
void enableDisplayOutput() {
  GPIOC->BSRR = ((DISPLAY_OE) << 16); // OE low => output enabled
}
  
void disableDisplayOutput() {
  GPIOC->BSRR = DISPLAY_OE;           // OE high => output disabled
}

// ----- Set Row Address -----
// Sets the row address using pins DISPLAY_A, B, C, D on Port D
void setRowAddress(uint8_t row) {
  uint32_t mask = (DISPLAY_A | DISPLAY_B | DISPLAY_C | DISPLAY_D);
  GPIOD->ODR &= ~mask;  // Clear current address bits
  
  if (row & 0x01) { GPIOD->BSRR = DISPLAY_A; }
  if (row & 0x02) { GPIOD->BSRR = DISPLAY_B; }
  if (row & 0x04) { GPIOD->BSRR = DISPLAY_C; }
  if (row & 0x08) { GPIOD->BSRR = DISPLAY_D; }
}

// ----- Refresh HUB75 Output -----
// Refreshes the entire HUB75 display using the ready frame in frameBuffer
void refreshHUB75Output() {
  disableDisplayOutput();
  
  // For a 32-line panel, refresh 16 pairs of lines (top and bottom)
  for (uint8_t row = 0; row < (NUM_ROWS / 2); row++) {
      setRowAddress(row);
      
      for (uint16_t col = 0; col < NUM_COLS; col++) {
          uint8_t wordIndex = col / 16;          // Each word contains 16 bits
          uint8_t bitPos = 15 - (col % 16);        // Extract bit from MSB to LSB
          
          uint8_t topPixel = (frameBuffer[readyFrameIndex][row][wordIndex] >> bitPos) & 0x01;
          uint8_t bottomPixel = (frameBuffer[readyFrameIndex][row + 16][wordIndex] >> bitPos) & 0x01;
          
          writePixelData(topPixel, bottomPixel);
          toggleDisplayClock();
      }
      
      toggleDisplayLatch();
      enableDisplayOutput();
      delayMicroseconds(20);
      disableDisplayOutput();
  }
  
  SerialDebug.println("Refreshed HUB75 with new frame data");
}

// ----- Setup and Loop -----
void setup() {
  SystemClock_Config();
  
  SerialDebug.begin(1000000);
  while (!SerialDebug) delay(10);
  SerialDebug.println("SerialDebug initialized");
  
  // Initialize DMD pins, SPI1 and HUB75 outputs
  DMD_InputPins_Init();
  SPI1_Slave_Bidirectional_Init();
  initHUB75Outputs();
  
  // Attach column latch (PB8) interrupt on rising edge
  attachInterrupt(digitalPinToInterrupt(PB8), columnStrobeISR, RISING);
  
  // Reset buffers (initialize frameBuffer to zero)
  spiByteIndex = 0;
  memset((void*)frameBuffer, 0, NUM_FRAME_BUFFERS * FRAME_LINES * WORD_BUFFER_SIZE * sizeof(uint16_t));
}
/*
void print128Binary(void) {
  char binString[129]; // 128 bits + null terminator
  int pos = 0;
  for (uint8_t i = 0; i < WORD_BUFFER_SIZE; i++) {
    uint16_t word = spi16Buffer[i];
    for (int bit = 15; bit >= 0; bit--) {
      binString[pos++] = (word & (1UL << bit)) ? '1' : '0';
    }
  }
  binString[pos] = '\0';
  SerialDebug.println(binString);
}
*/
void loop() {
  delay(1);
  
  if (newFrameReady) {
    refreshHUB75Output();
    newFrameReady = 0;
  }
}