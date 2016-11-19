/* Name: main.c
 * Author: Zack Lewis
 * License: <insert your license reference here>
 * Description: Reads value of 46 switches using a bank of
 *    74HC166 shift registers. Values are mapped to chars
 *    and output over USART.
 */

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FOSC 8000000 
#define BAUD 38400
#define MYUBRR FOSC/16/BAUD-1

/********************UART Stuff*****************/
void USART_init(uint16_t baud) { 
  /* Set baud rate */ 
  UBRRH = (unsigned char)(baud >> 8); 
  UBRRL = (unsigned char)baud; 
  /* Enable receiver, enable receive complete interrupt */ 
  UCSRB = (1 << RXEN)|(1 << TXEN)|(1 << RXCIE); 
  /* Set frame format: 8data, no parity, 2stop bit */ 
  UCSRC = (1 << USBS)|(1 << UCSZ0)|(1 << UCSZ1); 
}

void USART_transmit(unsigned char data) {
  while(!(UCSRA & (1 << UDRE)));
  UDR = data;
}

/********************SPI Stuff******************/
void pulse_clock() {
  PORTB |= (1 << PB4);
  _delay_us(10);
  PORTB &= ~(1 << PB4);
  _delay_us(10);  
}

void clock_high(time) {
  PORTB |= (1 << PB4);  
  _delay_us(10);
}

void clock_low(time) {
  PORTB &= ~(1 << PB4);  
  _delay_us(10);
}

void SPI_load() {
  //load low, bring clock high
  PORTB &= ~(1 << PB7);
  _delay_us(10);
  clock_high();
   //load high
  PORTB |= (1 << PB7);
  _delay_us(10);
  clock_low();

}

void SPI_init() {
  // CLK PB4
  // LD  PB7
  // QH  PB6
  DDRB = (1 << PB4)|(1 << PB7);
  PORTB = (1 << PB7);
}

uint8_t switchStates[2];

void loadSwitchStates() {
  uint8_t inByte;
  uint8_t i;
  uint8_t j;
  uint8_t serState;

  SPI_load();

  for (j = 0; j < 2; j++) {
    for (i = 0; i < 8; i++) {
      serState = (PINB & (1 << PB6)) >> 6;
      pulse_clock();
      inByte |= (serState << i);
    }
    switchStates[j] = inByte;
    inByte = 0;
  }
}

/**************EEPROM Stuff*********************/
void EEPROM_write(uint8_t uiAddress, uint8_t ucData){
  while(EECR & (1 << EEPE));

  EEAR = uiAddress;
  EEDR = ucData;

  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
}

uint8_t EEPROM_read(uint8_t uiAddress){
  while(EECR & (1 << EEPE));
  
  EEAR = uiAddress;
  EECR |= (1 << EERE);

  return EEDR;
}

/********************Other Stuff******************/

void transmitCharacter(key) {
  USART_transmit(key);
}

int8_t getActiveSwitch() {
  int8_t frame;
  int8_t pos;
  int8_t i;
  int8_t j;
  int8_t index = -1;
  //load register states into switch array
  loadSwitchStates();
  //loop through entire switch array
  for (i = 0; i < 2; i++) {
    if (switchStates[i] < 255) {
      frame = i;
      for (j = 0; j < 8; j++) {
        if (!(switchStates[frame] & (1 << j))) {
          pos = j;
          index = frame * 8 + pos;
        }
      }
    }
  }
  return index;
}

int8_t getKey() {
  int8_t key;
  int8_t switchIndex = getActiveSwitch();
  if (switchIndex >= 0) {
    _delay_ms(10);
    if (switchIndex == getActiveSwitch()) {
      key = EEPROM_read(switchIndex);
      return key;
    }
  }
  return -1;
}

void programKeys() {
  int8_t prompt[] = "Please press key ";
  int8_t characters[] = {'a', 'b', 'c', 'd', 'e', 'f',
          'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
          'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x',
          'y', 'z'};
  uint8_t i;
  int8_t input = -1;

  for (i = 0; i < 17; i++) {
    USART_transmit(prompt[i]);
  }
  for (i = 0; i < 16; i++) {
    USART_transmit(characters[i]);
    USART_transmit(':');
    while(input < 0) {
      input = getActiveSwitch();
    }
    _delay_ms(500);
    USART_transmit(input);
    USART_transmit('\n');
    EEPROM_write(input, characters[i]);
    input = -1;
  }
}



int main(void)
{
  int8_t keyPressed;
  int8_t previousKeyPressed = -1;

  USART_init(MYUBRR);  
  SPI_init();

//  programKeys();

  for(;;){
    keyPressed = getKey();
    if (keyPressed >= 0) {
      if (keyPressed != previousKeyPressed) {
        transmitCharacter(keyPressed);
        previousKeyPressed = keyPressed;
      }
    }
    previousKeyPressed = keyPressed;
  }
  return 0;   /* never reached */
}
