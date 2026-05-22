#0123456789012345678901234567890123456789012345678901234567890123456789012345678
# MorseLED.py
# Blocking functions for blinking the LED.  Do not use in a loop that requires
# time as this blocks until the entire message is sent.
# Copyright Paul Taylor, WC8T 2026

from machine import Pin
import time

# Morse code dictionary
MORSE_CODE = {
    'A': '.-',    'B': '-...',  'C': '-.-.', 'D': '-..',  'E': '.',
    'F': '..-.',  'G': '--.',   'H': '....', 'I': '..',   'J': '.---',
    'K': '-.-',   'L': '.-..',  'M': '--',   'N': '-.',   'O': '---',
    'P': '.--.',  'Q': '--.-',  'R': '.-.',  'S': '...',  'T': '-',
    'U': '..-',   'V': '...-',  'W': '.--',  'X': '-..-', 'Y': '-.--',
    'Z': '--..',
    '1': '.----', '2': '..---', '3': '...--', '4': '....-', '5': '.....',
    '6': '-....', '7': '--...', '8': '---..', '9': '----.', '0': '-----',
    ' ': ' '
}

# Adjustable WPM
WPM = 8  # Change this to adjust speed
DOT_DURATION = int(1200 / WPM)  # milliseconds per dot (PARIS standard)

def blink_dot(led):
    led.value(1)
    time.sleep_ms(DOT_DURATION)
    led.value(0)
    time.sleep_ms(DOT_DURATION)  # gap between elements

def blink_dash(led):
    led.value(1)
    time.sleep_ms(DOT_DURATION * 3)
    led.value(0)
    time.sleep_ms(DOT_DURATION)

def send_morse(message):
    # LED setup (onboard LED is Pin 25 on Pico)
    led = Pin("LED", Pin.OUT)
    
    for char in message.upper():
        code = MORSE_CODE.get(char, '')
        if code == ' ':
            time.sleep_ms(DOT_DURATION * 7)  # gap between words
        else:
            for symbol in code:
                if symbol == '.':
                    blink_dot(led)
                elif symbol == '-':
                    blink_dash(led)
            time.sleep_ms(DOT_DURATION * 2)  # gap between letters
