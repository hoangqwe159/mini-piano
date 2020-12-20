#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SET_BIT(reg, pin)		    (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)		  (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)   (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)		  (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)	     (BIT_VALUE((reg),(pin))==1)

#define LCD_USING_4PIN_MODE (1)
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define LCD_DATA4_DDR (DDRC)
#define LCD_DATA5_DDR (DDRC)
#define LCD_DATA6_DDR (DDRC)
#define LCD_DATA7_DDR (DDRC)

#define LCD_DATA4_PORT (PORTC)
#define LCD_DATA5_PORT (PORTC)
#define LCD_DATA6_PORT (PORTC)
#define LCD_DATA7_PORT (PORTC)

#define LCD_DATA4_PIN (2)
#define LCD_DATA5_PIN (3)
#define LCD_DATA6_PIN (4)
#define LCD_DATA7_PIN (5)


#define LCD_RS_DDR (DDRB)
#define LCD_ENABLE_DDR (DDRC)

#define LCD_RS_PORT (PORTB)
#define LCD_ENABLE_PORT (PORTC)

#define LCD_RS_PIN (3)
#define LCD_ENABLE_PIN (1)

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define BTN_C4_BIT 2
#define BTN_D4_BIT 3
#define BTN_E4_BIT 4
#define BTN_F4_BIT 5
#define BTN_G4_BIT 6
#define BTN_A4_BIT 7
#define BTN_B4_BIT 0
#define BTN_MENU_BIT 2

#define BTN_C4_DDR DDRD
#define BTN_D4_DDR DDRD
#define BTN_E4_DDR DDRD
#define BTN_F4_DDR DDRD
#define BTN_G4_DDR DDRD
#define BTN_A4_DDR DDRD
#define BTN_B4_DDR DDRB
#define BTN_MENU_DDR DDRB

#define BTN_C4_PORT PORTD
#define BTN_D4_PORT PORTD
#define BTN_E4_PORT PORTD
#define BTN_F4_PORT PORTD
#define BTN_G4_PORT PORTD
#define BTN_A4_PORT PORTD
#define BTN_B4_PORT PORTB
#define BTN_MENU_PORT PORTB

#define BTN_C4_PIN PIND
#define BTN_D4_PIN PIND
#define BTN_E4_PIN PIND
#define BTN_F4_PIN PIND
#define BTN_G4_PIN PIND
#define BTN_A4_PIN PIND
#define BTN_B4_PIN PINB
#define BTN_MENU_PIN PINB

#define LED_ACTIVE_BIT 4
#define LED_ACTIVE_DDR DDRB
#define LED_ACTIVE_PORT PORTB

//UART definitions
//define baud rate 
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

uint8_t mask = 0b00011111;
volatile uint8_t btn_menu_state = 0;
volatile uint8_t btn_menu_counter = 0;
volatile uint8_t pre_state = 0;
int menuPressedTimes = 0;

uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;

void debounce_setup(void);
void poten_setup(void);
uint16_t read_adc(void);
uint16_t read_adc(void);
int isNotePressed();
void LCDstart();
void setup(void);
void btn_setup(void);
void uart_init(unsigned int);
void uart_putchar(unsigned char data);
unsigned char uart_getchar(void);
void noteTone(int frequency, int duration);
void Song1(void);
void Song1Title(void);
void Song2(void);
void Song2Title(void);
void Song3(void);
void Song3Title(void);
void Song4(void);
void Song4Title(void);
void my_delay_ms(int ms);

void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);


void lcd_clear(void);
void lcd_home(void);

void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t); 

void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);

size_t lcd_write(uint8_t);
void lcd_command(uint8_t);


void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);


int main(void) {
    setup();
    const int noteDuartion = 50;
    while (1)
    {
        if (pre_state==0 && btn_menu_state == 1) {
            lcd_display();
            lcd_clear();
            LCDstart();
        }       
        
        if(!BIT_IS_SET(BTN_C4_PIN, BTN_C4_BIT)){              
            uart_putchar('C');
            while ( !BIT_IS_SET(BTN_C4_PIN, BTN_C4_BIT) ) {        
                noteTone(NOTE_C4,noteDuartion);  
            }              
        } else if ( !BIT_IS_SET(BTN_D4_PIN, BTN_D4_BIT)){              
            uart_putchar('D');
            while ( !BIT_IS_SET(BTN_D4_PIN, BTN_D4_BIT) ) {
            noteTone(NOTE_D4,noteDuartion);
            }
        } else if ( !BIT_IS_SET(BTN_E4_PIN, BTN_E4_BIT)){              
            uart_putchar('E');
            while ( !BIT_IS_SET(BTN_E4_PIN, BTN_E4_BIT) ) {
            noteTone(NOTE_E4,noteDuartion);
            }
        } else if ( !BIT_IS_SET(BTN_F4_PIN, BTN_F4_BIT)){              
            uart_putchar('F');
            while ( !BIT_IS_SET(BTN_F4_PIN, BTN_F4_BIT) ) {
            noteTone(NOTE_F4,noteDuartion);
            }
        } else if ( !BIT_IS_SET(BTN_G4_PIN, BTN_G4_BIT)){              
            uart_putchar('G');
            while ( !BIT_IS_SET(BTN_G4_PIN, BTN_G4_BIT) ) {
            noteTone(NOTE_G4,noteDuartion);
            }
        } else if ( !BIT_IS_SET(BTN_A4_PIN, BTN_A4_BIT)){              
            uart_putchar('A');
            while ( !BIT_IS_SET(BTN_A4_PIN, BTN_A4_BIT) ) {
            noteTone(NOTE_A4,noteDuartion);
            }
        } else if ( !BIT_IS_SET(BTN_B4_PIN, BTN_B4_BIT)){              
            uart_putchar('B');
            while ( !BIT_IS_SET(BTN_B4_PIN, BTN_B4_BIT) ) {
            noteTone(NOTE_B4,noteDuartion);
            }
        }      
    }
        
}

// Initialise Timer 0 in normal mode so that it overflows 
// with a period of approximately 0.004 seconds.
void debounce_setup(void) {    
    TCCR0B |= (1<<CS02);
    TIMSK0 |= (1<<TOIE0);
    sei();
}

// Setup potentiometer
void poten_setup(void){
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1);
}

// Read analog input 
uint16_t read_adc(void) {
    ADCSRA |= (1<<ADSC);
    while (!(ADCSRA & (1<<ADIF)));
    return ADCW;
}

// Check if any key note is pressed
int isNotePressed() {
    if (!BIT_IS_SET(BTN_C4_PIN, BTN_C4_BIT) 
            || !BIT_IS_SET(BTN_D4_PIN, BTN_D4_BIT) 
            || !BIT_IS_SET(BTN_E4_PIN, BTN_E4_BIT) 
            || !BIT_IS_SET(BTN_F4_PIN, BTN_F4_BIT) 
            || !BIT_IS_SET(BTN_G4_PIN, BTN_G4_BIT) 
            || !BIT_IS_SET(BTN_A4_PIN, BTN_A4_BIT) 
            || !BIT_IS_SET(BTN_B4_PIN, BTN_B4_BIT)) {
                return 1;
            }
    return 0;
}

// ISR for Timer 0
ISR(TIMER0_OVF_vect){
    pre_state = btn_menu_state;
    btn_menu_counter = btn_menu_counter << 1;
    btn_menu_counter = btn_menu_counter & (mask);
    btn_menu_counter = btn_menu_counter | !BIT_VALUE(BTN_MENU_PIN, BTN_MENU_BIT);
    if(btn_menu_counter == mask){
        btn_menu_state = 1;
    }else if (btn_menu_counter == 0){
        btn_menu_state = 0;
    }
}

// Start and display menu
void LCDstart(){
    menuPressedTimes = 0;
    LED_ACTIVE_PORT |= (1 << LED_ACTIVE_BIT);
    lcd_clear();
    lcd_write_string(0,0, "WHICH SONG WOULD");
    lcd_write_string(0,1, "YOU LIKE TO PLAY");
    _delay_ms(50);

    while (menuPressedTimes == 0)  { // continue loop while menu button isn't pressed
        if(pre_state==0 && btn_menu_state == 1) {
            menuPressedTimes++;
            _delay_ms(200);
            Song1Title();
            lcd_clear();
            break;
        }      
    } // end while    
} // end LCDstart loop

void setup(void){
    DDRB |= (1<<1);
    btn_setup();
    poten_setup();
    debounce_setup();
    lcd_init();
    uart_init(MYUBRR);
}

// Setup buttons
void btn_setup(void){
    DDRD &= ~((1<<BTN_C4_BIT)|(1<<BTN_D4_BIT)|(1<<BTN_E4_BIT)|(1<<BTN_F4_BIT)|(1<<BTN_G4_BIT)|(1<<BTN_A4_BIT));          
    PORTD |= (1<<BTN_C4_BIT)|(1<<BTN_D4_BIT)|(1<<BTN_E4_BIT)|(1<<BTN_F4_BIT)|(1<<BTN_G4_BIT)|(1<<BTN_A4_BIT) ;

    BTN_B4_DDR &= ~((1<<BTN_B4_BIT));
    BTN_B4_PORT |= (1<<BTN_B4_BIT);

    BTN_MENU_DDR &= ~((1<< BTN_MENU_BIT));
    BTN_MENU_PORT |= (1<<BTN_MENU_BIT);
    LED_ACTIVE_DDR &= ~(1<< LED_ACTIVE_BIT);
}

// Initialise uart
void uart_init(unsigned int ubrr){
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (3 << UCSZ00);
    sei();
}

// Transmit data
void uart_putchar(unsigned char data){
    while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/            
    UDR0 = data;            /* Put data into buffer, sends the data */        
}

// Receive data
unsigned char uart_getchar(void){
    /* Wait for data to be received */
    while ( !(UCSR0A & (1<<RXC0)) );
    return UDR0;    
}

// Play note
void noteTone(int frequency, int duration){
    uint16_t adc_value = read_adc();
    float vol = adc_value * (5.0 / 1023.0);

    int a =16000000 / frequency / 8 ;  
    TCCR1B |= (1<<CS11);
    ICR1 = a;
    TCCR1A |= (1<<COM1A1) | (1<<WGM11);
    TCCR1B |= (1<<WGM12) | (1<<WGM13);
    OCR1A =  a/2 * (vol / 5);
    my_delay_ms(duration);
    OCR1A = 0;
}

// Delay the program
void my_delay_ms(int ms)
{
  while (0 < ms)
  {  
    _delay_ms(1);
    --ms;
  }
}

// Play MARIO THEME SONG
void Song1(void)  {  
    lcd_clear();
    // notes in the melody:
    int melody[] = {
        NOTE_E7, NOTE_E7, 0, NOTE_E7,
        0, NOTE_C7, NOTE_E7, 0,
        NOTE_G7, 0, 0,  0,
        NOTE_G6, 0, 0, 0,

        NOTE_C7, 0, 0, NOTE_G6,
        0, 0, NOTE_E6, 0,
        0, NOTE_A6, 0, NOTE_B6,
        0, NOTE_AS6, NOTE_A6, 0,

        NOTE_G6, NOTE_E7, NOTE_G7,
        NOTE_A7, 0, NOTE_F7, NOTE_G7,
        0, NOTE_E7, 0, NOTE_C7,
        NOTE_D7, NOTE_B6, 0, 0,

        NOTE_C7, 0, 0, NOTE_G6,
        0, 0, NOTE_E6, 0,
        0, NOTE_A6, 0, NOTE_B6,
        0, NOTE_AS6, NOTE_A6, 0,

        NOTE_G6, NOTE_E7, NOTE_G7,
        NOTE_A7, 0, NOTE_F7, NOTE_G7,
        0, NOTE_E7, 0, NOTE_C7,
        NOTE_D7, NOTE_B6, 0, 0
    };

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,

        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,

        9, 9, 9,
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,

        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,

        9, 9, 9,
        12, 12, 12, 12,
        12, 12, 12, 12,
        12, 12, 12, 12,
    }; 

    lcd_write_string(1,0,"Now playing...");
    lcd_write_string(0,1,"MARIO THEME SONG");

    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {
        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote] / 3;

        noteTone( melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 0.5;
        my_delay_ms(pauseBetweenNotes);

        // stop the tone playing:
        noteTone( 0, noteDuration); 
    }
    Song1Title();
  
  } // end Song1 loop

// MARIO THEME SONG menu
void  Song1Title(void){
    lcd_clear();
    lcd_write_string(0,0,"SUPER MARIO BROS");
    lcd_write_string(0,1,"   THEME SONG");
    while (menuPressedTimes == 1) {
        if (pre_state==0 && btn_menu_state == 1) {
            menuPressedTimes++;
            LED_ACTIVE_PORT |= (1 << LED_ACTIVE_BIT);
            _delay_ms(200);
            Song2Title();
            lcd_clear();
        }        
        if (isNotePressed()) {
            Song1();
            break;
        } 
    }    
}

// Play SUPER MARIO BROS UNDERWORD
void Song2(void) {
    lcd_clear();
    // notes in the melody:
    int melody[] = {
    NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
    NOTE_AS3, NOTE_AS4, 0,
    0,
    NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
    NOTE_AS3, NOTE_AS4, 0,
    0,
    NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
    NOTE_DS3, NOTE_DS4, 0,
    0,
    NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
    NOTE_DS3, NOTE_DS4, 0,
    0, NOTE_DS4, NOTE_CS4, NOTE_D4,
    NOTE_CS4, NOTE_DS4,
    NOTE_DS4, NOTE_GS3,
    NOTE_G3, NOTE_CS4,
    NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
    NOTE_GS4, NOTE_DS4, NOTE_B3,
    NOTE_AS3, NOTE_A3, NOTE_GS3,
    0, 0, 0
    };

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
        12, 12, 12, 12,
        12, 12, 6,
        3,
        12, 12, 12, 12,
        12, 12, 6,
        3,
        12, 12, 12, 12,
        12, 12, 6,
        3,
        12, 12, 12, 12,
        12, 12, 6,
        6, 18, 18, 18,
        6, 6,
        6, 6,
        6, 6,
        18, 18, 18, 18, 18, 18,
        10, 10, 10,
        10, 10, 10,
        3, 3, 3
    };


    lcd_write_string(1,0,"Now playing...");
    lcd_write_string(0,1,"MARIO UNDERWORLD");
     
      


    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {

        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote]/4;

        noteTone( melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 0.5;
        my_delay_ms(pauseBetweenNotes);

        // stop the tone playing:
        noteTone( 0, noteDuration);     
 
    }   
    Song2Title();
}

// SUPER MARIO BROS UNDERWORD menu  
void Song2Title(void) {
    lcd_clear();
    lcd_write_string(0,0,"SUPER MARIO BROS");
    lcd_write_string(0,1,"   UNDERWORLD");
    while (menuPressedTimes == 2) {
        if (pre_state==0 && btn_menu_state == 1) {
            menuPressedTimes++;
            LED_ACTIVE_PORT |= (1 << LED_ACTIVE_BIT);
            _delay_ms(200);
            Song3Title();
            lcd_clear();
        }        
        if (isNotePressed()) {
            Song2();
            break;
        } 
    }  
}

// Play JEOPARDY THEME SONG
void Song3(void) {
lcd_clear();


// notes in the melody:
int melody[] = {
	NOTE_C2, NOTE_F3, NOTE_C3, NOTE_A2, NOTE_A2,
	NOTE_C3, NOTE_F3, NOTE_C3,
	NOTE_C3, NOTE_F3, NOTE_C3, NOTE_F3,
	NOTE_AS3, NOTE_G3, NOTE_F3, NOTE_E3, NOTE_D3, NOTE_CS3,
	NOTE_C2, NOTE_F3, NOTE_C3, NOTE_A2, NOTE_A2, // repeat line 1 and 2
	NOTE_C3, NOTE_F3, NOTE_C3,
	NOTE_AS3, 0, NOTE_G3, NOTE_F3,
	NOTE_E3, NOTE_D3, NOTE_CS3, NOTE_C3};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
	4,    4,    4,    8,   8,
	4,    4,          2,
	4,    4,    4,    4,
	3,    8,    8,    8,    8,   8,
	4,    4,    4,    8,    8, // repeat line 1 and 2
	4,    4,          2,
	4,    8,    8,    4,    4,
	4,    4,    4,    4,
	0};


    lcd_write_string(1,0,"Now playing...");
    lcd_write_string(0,1,"    JEOPARDY");
     
      


    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {

      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 2000 / noteDurations[thisNote]/4;

      noteTone( melody[thisNote], noteDuration);

      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 0.2;
      my_delay_ms(pauseBetweenNotes);
 
      // stop the tone playing:
      noteTone( 0, noteDuration);     
 
    }   


    Song3Title();
}

// JEOPARDY THEME SONG menu
void Song3Title(void) {
    lcd_clear();
    lcd_write_string(0,0,"    JEOPARDY");
    lcd_write_string(0,1,"   THEME SONG");
    while (menuPressedTimes == 3) {
        if (pre_state==0 && btn_menu_state == 1) {
            menuPressedTimes++;
            LED_ACTIVE_PORT |= (1 << LED_ACTIVE_BIT);
            _delay_ms(200);
            Song4Title();
            lcd_clear();
        }        
        if (isNotePressed()) {
            Song3();
            break;
        } 
    }  
}

// Play A-ha! Take on Me THEME SONG
void Song4(void) {
    lcd_clear();


    // The melody array 
    int melody[] = {
        NOTE_FS5, NOTE_FS5, NOTE_D5, NOTE_B4, NOTE_B4, NOTE_E5, 
        NOTE_E5, NOTE_E5, NOTE_GS5, NOTE_GS5, NOTE_A5, NOTE_B5, 
        NOTE_A5, NOTE_A5, NOTE_A5, NOTE_E5, NOTE_D5, NOTE_FS5, 
        NOTE_FS5, NOTE_FS5, NOTE_E5, NOTE_E5, NOTE_FS5, NOTE_E5
    };

    // The note duration, 8 = 8th note, 4 = quarter note, etc.
    int noteDurations[] = {
        8, 8, 8, 4, 4, 4, 
        4, 5, 8, 8, 8, 8, 
        8, 8, 8, 4, 4, 4, 
        4, 5, 8, 8, 8, 8
    };


    lcd_write_string(1,0,"Now playing...");
    lcd_write_string(0,1,"  A-ha! Take on Me");
     
      


    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {

        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote]/2;

        noteTone( melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 0.5;
        my_delay_ms(pauseBetweenNotes);

        // stop the tone playing:
        noteTone( 0, noteDuration);     
 
    }   


    Song4Title();
}

// A-ha! Take on Me THEME SONG menu
void Song4Title(void) {
    lcd_clear();
    lcd_write_string(0,0,"A-ha! Take on Me");
    lcd_write_string(0,1,"   THEME SONG");
    while (menuPressedTimes == 4) {
        if (pre_state==0 && btn_menu_state == 1) {
            menuPressedTimes++;
            LED_ACTIVE_PORT &= ~(1 << LED_ACTIVE_BIT);
            _delay_ms(200);            
            lcd_clear();
        }        
        if (isNotePressed()) {
            Song4();
            break;
        } 
    }  
}

void lcd_init(void){
//dotsize
if (LCD_USING_4PIN_MODE){
    _lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
} else {
    _lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
}

_lcd_displayfunction |= LCD_2LINE;

// RS Pin
LCD_RS_DDR |= (1 << LCD_RS_PIN);
// Enable Pin
LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);

#if LCD_USING_4PIN_MODE
    //Set DDR for all the data pins
    LCD_DATA4_DDR |= (1 << 4);
    LCD_DATA5_DDR |= (1 << 5);
    LCD_DATA6_DDR |= (1 << 6);    
    LCD_DATA7_DDR |= (1 << 7);

#else
    //Set DDR for all the data pins
    LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
    LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
    LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
    LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
    LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
    LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
    LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
    LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
#endif 

// SEE PAGE 45/46 OF Hitachi HD44780 DATASHEET FOR INITIALIZATION SPECIFICATION!

// according to datasheet, we need at least 40ms after power rises above 2.7V
// before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
_delay_us(50000); 
// Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);

//put the LCD into 4 bit or 8 bit mode
if (LCD_USING_4PIN_MODE) {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms
    
    // third go!
    lcd_write4bits(0b0111); 
    _delay_us(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0b0010); 
} else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(4500);  // wait more than 4.1ms

    // second try
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
}

// finally, set # lines, font size, etc.
lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);  

// turn the display on with no cursor or blinking default
_lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
lcd_display();

// clear it off
lcd_clear();

// Initialize to default text direction (for romance languages)
_lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
// set the entry mode
lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[]){
lcd_setCursor(x,y);
for(int i=0; string[i]!='\0'; ++i){
    lcd_write(string[i]);
}
}

void lcd_write_char(uint8_t x, uint8_t y, char val){
lcd_setCursor(x,y);
lcd_write(val);
}

void lcd_clear(void){
lcd_command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
_delay_us(2000);  // this command takes a long time!
}

void lcd_home(void){
lcd_command(LCD_RETURNHOME);  // set cursor position to zero
_delay_us(2000);  // this command takes a long time!
}


// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
location &= 0x7; // we only have 8 locations 0-7
lcd_command(LCD_SETCGRAMADDR | (location << 3));
for (int i=0; i<8; i++) {
    lcd_write(charmap[i]);
}
}

void lcd_setCursor(uint8_t col, uint8_t row){
if ( row >= 2 ) {
    row = 1;
}

lcd_command(LCD_SETDDRAMADDR | (col + row*0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void) {
_lcd_displaycontrol &= ~LCD_DISPLAYON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_display(void) {
_lcd_displaycontrol |= LCD_DISPLAYON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void) {
_lcd_displaycontrol &= ~LCD_CURSORON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_cursor(void) {
_lcd_displaycontrol |= LCD_CURSORON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void) {
_lcd_displaycontrol &= ~LCD_BLINKON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

void lcd_blink(void) {
_lcd_displaycontrol |= LCD_BLINKON;
lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void scrollDisplayRight(void) {
lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
_lcd_displaymode |= LCD_ENTRYLEFT;
lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
_lcd_displaymode &= ~LCD_ENTRYLEFT;
lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
_lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
_lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
//
lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value) {
lcd_send(value, 1);
return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode) {
//RS Pin
LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
LCD_RS_PORT |= (!!mode << LCD_RS_PIN);

if (LCD_USING_4PIN_MODE) {
    lcd_write4bits(value>>4);
    lcd_write4bits(value);
} else {
    lcd_write8bits(value); 
} 
}

void lcd_pulseEnable(void) {
//Enable Pin
LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
_delay_us(1);    
LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
_delay_us(1);    // enable pulse must be >450ns
LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
_delay_us(100);   // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value) {
//Set each wire one at a time

LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
value >>= 1;

LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
value >>= 1;

LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
value >>= 1;

LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value) {
//Set each wire one at a time

#if !LCD_USING_4PIN_MODE
    LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
    LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
    value >>= 1;

    LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
    LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
    value >>= 1;

    LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
    LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
    value >>= 1;

    LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
    LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
    value >>= 1;

    LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
    LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
    value >>= 1;

    LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
    LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
    value >>= 1;

    LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
    LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
    value >>= 1;

    LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
    LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);
    
    lcd_pulseEnable();
#endif
}



