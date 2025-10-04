/************************************************************************
  main.c

  Main functions
    RP2040 - USB to quadrature mouse and keyboard converter
    Copyright (C) 2025 Jose I. Romero
    Copyright (C) 2023 Darren Jones
    Copyright (C) 2017-2020 Simon Inns

  This file is part of RP2040 Mouse based on the original SmallyMouse from Simon Inns.

    RP2040 Mouse is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Email: nz.darren.jones@gmail.com

************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"

// Override default USB pin
#define PIO_USB_DP_PIN_DEFAULT 8

// Configure RP2040 for slower flash
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#define PICO_BOOT_STAGE2_CHOOSE_GENERIC_03H 1

#define VERSION "1.0"

#include "pio_usb.h"
#include "tusb.h"
#include "main.h"

#include "hardware/uart.h"
#include "hardware/clocks.h"
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 12
#define UART_RX_PIN 13

#ifdef DEBUG
#define DEBUG_PRINT(x) printf x
#define CFG_TUSB_DEBUG 3
#else
#define DEBUG_PRINT(x) \
  do                   \
  {                    \
  } while (0)
#define CFG_TUSB_DEBUG 0
#endif


enum KBDCommState {
  KBD_IDLE,
  KBD_WAIT_TO_RECEIVE,
  KBD_READING_COMMAND,
  KBD_DONE_READING,
  KBD_WAIT_TO_SEND,
  KBD_SENDING_RESPONSE
};

enum KBDCommands {
  KBD_CMD_INQUIRY = 0x10,
  KBD_CMD_INSTANT = 0x14,
  KBD_CMD_GET_MODEL = 0x16,
  KBD_CMD_TEST = 0x36
};

const uint8_t kbdTestResponse = 0x7D;
const uint8_t kbdModel = 0x0B; // M0110A Keyboard
const uint32_t idleTimeout = 250 / 0.2; // 250ms timeout


enum KBDCommState kbdCommState = KBD_IDLE;
bool kbdClockPhase = false;
uint8_t kbdShiftBuffer = 0;
uint8_t kbdShiftBitCount = 0;
// Keyboard event queue
uint8_t keyevent_queue[16];
uint8_t keyevent_head = 0;
uint8_t keyevent_tail = 0;
// Idle timeout
uint32_t idleTime = 0;

hid_keyboard_report_t prev_keyboard_state = {0};

const uint8_t hid_to_m01100a[256] = {
  [HID_KEY_GRAVE] = 0x65,
  [HID_KEY_1] = 0x25,
  [HID_KEY_2] = 0x27,
  [HID_KEY_3] = 0x29,
  [HID_KEY_4] = 0x2B,
  [HID_KEY_5] = 0x2F,
  [HID_KEY_6] = 0x2D,
  [HID_KEY_7] = 0x35,
  [HID_KEY_8] = 0x39,
  [HID_KEY_9] = 0x33,
  [HID_KEY_0] = 0x3B,
  [HID_KEY_MINUS] = 0x37,
  [HID_KEY_EQUAL] = 0x31,
  [HID_KEY_BACKSPACE] = 0x67,

  [HID_KEY_TAB] = 0x61,
  [HID_KEY_Q] = 0x19,
  [HID_KEY_W] = 0x1B,
  [HID_KEY_E] = 0x1D,
  [HID_KEY_R] = 0x1F,
  [HID_KEY_T] = 0x23,
  [HID_KEY_Y] = 0x21,
  [HID_KEY_U] = 0x41,
  [HID_KEY_I] = 0x45,
  [HID_KEY_O] = 0x3F,
  [HID_KEY_P] = 0x47,
  [HID_KEY_BRACKET_LEFT] = 0x43,
  [HID_KEY_BRACKET_RIGHT] = 0x3D,

  [HID_KEY_CAPS_LOCK] = 0x73,
  [HID_KEY_A] = 0x01,
  [HID_KEY_S] = 0x03,
  [HID_KEY_D] = 0x05,
  [HID_KEY_F] = 0x07,
  [HID_KEY_G] = 0x0B,
  [HID_KEY_H] = 0x09,
  [HID_KEY_J] = 0x4D,
  [HID_KEY_K] = 0x51,
  [HID_KEY_L] = 0x4B,
  [HID_KEY_SEMICOLON] = 0x53,
  [HID_KEY_APOSTROPHE] = 0x4F,
  [HID_KEY_ENTER] = 0x49,

  [HID_KEY_SHIFT_LEFT] = 0x71,
  [HID_KEY_Z] = 0x0D,
  [HID_KEY_X] = 0x0F,
  [HID_KEY_C] = 0x11,
  [HID_KEY_V] = 0x13,
  [HID_KEY_B] = 0x17,
  [HID_KEY_N] = 0x5B,
  [HID_KEY_M] = 0x5D,
  [HID_KEY_COMMA] = 0x27,
  [HID_KEY_PERIOD] = 0x5F,
  [HID_KEY_SLASH] = 0x59,
  [HID_KEY_SHIFT_RIGHT] = 0x71,
  
  [HID_KEY_CONTROL_LEFT] = 0x6F,
  [HID_KEY_CONTROL_RIGHT] = 0x6F,
  [HID_KEY_GUI_LEFT] = 0x6F,
  [HID_KEY_GUI_RIGHT] = 0x6F,
  [HID_KEY_ALT_LEFT] = 0x75,
  [HID_KEY_ALT_RIGHT] = 0x75,
  [HID_KEY_SPACE] = 0x63,
  [HID_KEY_BACKSLASH] = 0x55,
  // Arrows
  [HID_KEY_ARROW_UP] = 0x80 | 0x1B,
  [HID_KEY_ARROW_DOWN] = 0x80 | 0x11,
  [HID_KEY_ARROW_LEFT] = 0x80 | 0x0D,
  [HID_KEY_ARROW_RIGHT] = 0x80 | 0x05,

  // Keypad
  [HID_KEY_KEYPAD_0] = 0x80 | 0x25,
  [HID_KEY_KEYPAD_1] = 0x80 | 0x27,
  [HID_KEY_KEYPAD_2] = 0x80 | 0x29,
  [HID_KEY_KEYPAD_3] = 0x80 | 0x2B,
  [HID_KEY_KEYPAD_4] = 0x80 | 0x2D,
  [HID_KEY_KEYPAD_5] = 0x80 | 0x2F,
  [HID_KEY_KEYPAD_6] = 0x80 | 0x31,
  [HID_KEY_KEYPAD_7] = 0x80 | 0x33,
  [HID_KEY_KEYPAD_8] = 0x80 | 0x37,
  [HID_KEY_KEYPAD_9] = 0x80 | 0x39,
  [HID_KEY_KEYPAD_ENTER] = 0x80 | 0x19,
  [HID_KEY_KEYPAD_DECIMAL] = 0x80 | 0x03,
  [HID_KEY_KEYPAD_CLEAR_ENTRY] = 0x80 | 0x0F,
  [HID_KEY_KEYPAD_SUBTRACT] = 0x80 | 0x1D,
};


/**
 * @brief Enqueues a keyboard event for processing.
 *
 * This function adds the specified keyboard event to the event queue.
 * The event parameter typically represents a key press or release,
 * encoded as a uint8_t value.
 *
 * @param event The keyboard event to enqueue.
 */
#include "hardware/sync.h"

void kbd_enqueue(uint8_t event) {
  uint32_t irq_state = save_and_disable_interrupts();
  uint8_t next_head = (keyevent_head + 1) % sizeof(keyevent_queue);
  if (next_head != keyevent_tail) { // Check for queue full
    keyevent_queue[keyevent_head] = event;
    keyevent_head = next_head;
  }
  restore_interrupts(irq_state);
}

/**
 * @brief Dequeues a keyboard event for processing.
 *
 * This function retrieves and removes the next keyboard event from the event queue.
 * If the queue is empty, it returns 0.
 *
 * @return The next keyboard event, or 0 if the queue is empty.
 */
uint8_t kbd_dequeue() {
  uint32_t irq_state = save_and_disable_interrupts();
  if (keyevent_head == keyevent_tail) {
    restore_interrupts(irq_state);
    return 0; // Queue empty
  }
  uint8_t event = keyevent_queue[keyevent_tail];
  keyevent_tail = (keyevent_tail + 1) % sizeof(keyevent_queue);
  restore_interrupts(irq_state);
  return event;
}

void kbd_enqueue_scancode(uint8_t scancode, uint8_t pressed) {
  uint8_t mac_code = hid_to_m01100a[scancode];
  uint8_t pressed_mask = pressed ? 0x00 : 0x80;
  if (mac_code == 0) {
    switch (scancode) { // Check for the special shifted keypad keys
      case HID_KEY_KEYPAD_EQUAL:
        kbd_enqueue(pressed_mask | 0x71); // Shift
        kbd_enqueue(0x79);
        kbd_enqueue(pressed_mask | 0x11);
        break;
      case HID_KEY_KEYPAD_ADD:
        kbd_enqueue(pressed_mask | 0x71);
        kbd_enqueue(0x79);
        kbd_enqueue(pressed_mask | 0x0D);
        break;
      case HID_KEY_KEYPAD_MULTIPLY:
        kbd_enqueue(pressed_mask | 0x71);
        kbd_enqueue(0x79);
        kbd_enqueue(pressed_mask | 0x05);
        break;
      case HID_KEY_KEYPAD_DIVIDE:
        kbd_enqueue(pressed_mask | 0x71);
        kbd_enqueue(0x79);
        kbd_enqueue(pressed_mask | 0x1B);
        break;
      default:
        return; // No mapping for this key, ignore it
    }
  } else if (mac_code & 0x80) {
    // Key is a keypad key. send 0x79 before it
    kbd_enqueue(0x79);
    kbd_enqueue(pressed_mask | (mac_code & 0x7F));
  } else {
    kbd_enqueue(pressed_mask | mac_code);
  }
}

// Quadrature output buffer limit
//
// Since the slow rate limit will prevent the quadrature output keeping up with the USB movement
// report input, the quadrature output will lag behind (i.e. the quadrature mouse will continue
// to move after the USB mouse has stopped).  This setting limits the maximum number of buffered
// movements to the quadrature output.  If the buffer reaches this value further USB movements
// will be discarded
#define Q_BUFFERLIMIT 300

// Interrupt Service Routines for quadrature output -------------------------------------------------------------------

// The following globals are used by the interrupt service routines to track the mouse
// movement.  The mouseDirection indicates which direction the mouse is moving in and
// the mouseEncoderPhase tracks the current phase of the quadrature output.
//
// The mouseDistance variable tracks the current distance the mouse has left to move
// (this is incremented by the USB mouse reports and decremented by the ISRs as they
// output the quadrature to the retro host).
volatile int8_t mouseEncoderPhaseX = 0; // X Quadrature phase (0-3)
volatile int8_t mouseEncoderPhaseY = 0; // Y Quadrature phase (0-3)

volatile int16_t mouseDistanceX = 0; // Distance left for mouse to move
volatile int16_t mouseDistanceY = 0; // Distance left for mouse to move

struct repeating_timer timer1;



void core1_main()
{
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
  tuh_hid_set_default_protocol(HID_PROTOCOL_REPORT);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true)
  {
    tuh_task(); // tinyusb host task
  }
}

void kbd_send_byte(uint8_t data) {
  kbdShiftBuffer = data;
  kbdCommState = KBD_SENDING_RESPONSE;
  kbdShiftBitCount = 8; // Need to send 8 bits
}

bool timer1_callback(struct repeating_timer * /* t */)
{
  // Process X output
  if (mouseDistanceX != 0)
  {
    if (mouseDistanceX < 0){
      mouseEncoderPhaseX = (mouseEncoderPhaseX + 3) % 4;
      mouseDistanceX++;
    } else if (mouseDistanceX > 0){
      mouseEncoderPhaseX = (mouseEncoderPhaseX + 1) % 4;
      mouseDistanceX--;
    }

    switch (mouseEncoderPhaseX)
    {
    case 0:
      gpio_put(XA_PIN, 1);
      gpio_put(XB_PIN, 0);
      break;
    case 1:
      gpio_put(XA_PIN, 1);
      gpio_put(XB_PIN, 1);
      break;
    case 2:
      gpio_put(XA_PIN, 0);
      gpio_put(XB_PIN, 1);
      break;
    case 3:
    default:
      gpio_put(XA_PIN, 0);
      gpio_put(XB_PIN, 0);
    }
  }
  // Process Y output
  if (mouseDistanceY != 0)
  {
    if (mouseDistanceY < 0){
      mouseEncoderPhaseY = (mouseEncoderPhaseY + 3) % 4;
      mouseDistanceY++;
    } else if (mouseDistanceY > 0){
      mouseEncoderPhaseY = (mouseEncoderPhaseY + 1) % 4;
      mouseDistanceY--;
    }
    switch (mouseEncoderPhaseY) // Y Axis is reversed
    {
    case 0:
      gpio_put(YA_PIN, 1);
      gpio_put(YB_PIN, 0);
      break;
    case 1:
      gpio_put(YA_PIN, 0);
      gpio_put(YB_PIN, 0);
      break;
    case 2:
      gpio_put(YA_PIN, 0);
      gpio_put(YB_PIN, 1);
      break;
    case 3:
    default:
      gpio_put(YA_PIN, 1);
      gpio_put(YB_PIN, 1);
    }
  }
  switch (kbdCommState) {
    case KBD_IDLE:
      gpio_set_dir(KBD_DATA_PIN, GPIO_IN);
      gpio_put(KBD_CLK_PIN, 1);
      // On data falling edge, start receiving
      if (!gpio_get(KBD_DATA_PIN)) {
        kbdCommState = KBD_WAIT_TO_RECEIVE;
      }
      break;
    case KBD_WAIT_TO_RECEIVE:
      kbdShiftBitCount = 8; // Need to read 8 bits
      kbdClockPhase = false; // Start on clock falling edge
      gpio_put(KBD_CLK_PIN, 0);
      kbdCommState = KBD_READING_COMMAND;
      break;
    case KBD_READING_COMMAND:
      if (!kbdClockPhase) {
        // On clock low phase, set clock high and sample data line
        gpio_put(KBD_CLK_PIN, 1);
        kbdClockPhase = true;
        kbdShiftBuffer <<= 1;
        if (gpio_get(KBD_DATA_PIN)) {
          kbdShiftBuffer |= 0x01;
        }
      } else {
        // On clock high phase, set clock low if more bits to read
        if (--kbdShiftBitCount) {
          // Start next clock cycle
          gpio_put(KBD_CLK_PIN, 0);
          kbdClockPhase = false;
        } else {
          // Command byte received, send response when macintosh is ready (data line high)
          kbdCommState = KBD_DONE_READING;
        }
      }
      break;
    case KBD_DONE_READING:
      // Macintosh is ready, decide what to send based on command
      if (gpio_get(KBD_DATA_PIN)) {
        kbdCommState = KBD_WAIT_TO_SEND;
      }
      idleTime = 0;
      break;
    case KBD_WAIT_TO_SEND:
      switch (kbdShiftBuffer) {
        case KBD_CMD_INQUIRY: { // Blocking request for key event
          uint8_t event = kbd_dequeue();
          if (event) { // Only respond if there is an event in the queue
            kbd_send_byte(event); // Send next key event
          } else {
            idleTime++;
            if (idleTime >= idleTimeout) {
              kbd_send_byte(0x7B); // Send null event if timed out
            }
          }
          break;
        }
        case KBD_CMD_INSTANT: { // Non-blocking request for key event
          uint8_t event = kbd_dequeue(); // Send next key event or 0 if none
          if (!event) {
            event = 0x7B; // Null event
          }
          kbd_send_byte(event);
          break;
        }
        case KBD_CMD_GET_MODEL:
          kbd_send_byte(kbdModel); // Macintosh Plus Keyboard
          break;
        case KBD_CMD_TEST:
          kbd_send_byte(kbdTestResponse); // Always respond with 0x7D
          break;
        default:
          kbd_send_byte(0x7B); // Unknown command
          break;
      }
      break;
    case KBD_SENDING_RESPONSE:
      if (!kbdClockPhase) {
        // On clock low phase, set clock high
        gpio_put(KBD_CLK_PIN, 1);
        kbdClockPhase = true;
        if (--kbdShiftBitCount <= 0) {
          // All bits sent, go back to waiting for next command
          kbdCommState = KBD_IDLE;
          gpio_set_dir(KBD_DATA_PIN, GPIO_IN); // Release data line
        }
      } else {
        // On clock high phase, set clock low and shift out most significant bit
        if(kbdShiftBuffer & 0x80) {
          gpio_set_dir(KBD_DATA_PIN, GPIO_IN); // Release data line
        } else {
          gpio_set_dir(KBD_DATA_PIN, GPIO_OUT);
          gpio_put(KBD_DATA_PIN, 0); // Pull data line low
        }
        kbdShiftBuffer <<= 1;
        gpio_put(KBD_CLK_PIN, 0);
        kbdClockPhase = false;
      }
      break;
  }

  return true;
}

static void blink_status(uint8_t count)
{
  uint8_t i = 0;
  gpio_put(STATUS_PIN, 0);

  while (i < count)
  {
    sleep_ms(200);
    gpio_put(STATUS_PIN, 1);
    sleep_ms(200);
    gpio_put(STATUS_PIN, 0);
    i++;
  }
}

int main()
{
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);
  stdio_init_all();

// Setup Debug to UART
#ifdef DEBUG
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_init(UART_ID, 2400);
  int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
#endif
  DEBUG_PRINT(("\033[2J"));
  DEBUG_PRINT(("****************************************************\r\n"));
  DEBUG_PRINT(("*         RP2040 USB To Quadrature Adapter         *\r\n"));
  DEBUG_PRINT(("*         Copyright 2022 Darren Jones              *\r\n"));
  DEBUG_PRINT(("*         (nz.darren.jones@gmail.com)              *\r\n"));
  DEBUG_PRINT(("*         Version: %s                             *\r\n", VERSION));
  DEBUG_PRINT(("*         Build Date: %s %s         *\r\n", __DATE__, __TIME__));
  DEBUG_PRINT(("****************************************************\r\n"));
  DEBUG_PRINT(("\r\n"));
  DEBUG_PRINT(("RP2040 USB To Quadrature Booting.....\r\n"));

  // all USB task run in core1
  multicore_reset_core1();
  DEBUG_PRINT(("Core1 Reset\r\n"));

  multicore_launch_core1(core1_main);
  DEBUG_PRINT(("Core1 Launched\r\n"));

  // Initialise the RP2040 hardware
  initialiseHardware();
  add_repeating_timer_us(-200, timer1_callback, NULL, &timer1);
  DEBUG_PRINT(("Hardware Initalized\r\n"));

  // Blink Status LED and wait for everything to settle
  blink_status(10);

  // Initialise the timers
  while (true)
  {
    stdio_flush();
    sleep_us(10);
  }
}

void initialiseHardware(void)
{
  // Document pins for picotool
  bi_decl(bi_1pin_with_name(XA_PIN, "X1 Quadrature Output"));
  bi_decl(bi_1pin_with_name(XB_PIN, "X2 Quadrature Output"));
  bi_decl(bi_1pin_with_name(YA_PIN, "Y1 Quadrature Output"));
  bi_decl(bi_1pin_with_name(YB_PIN, "Y2 Quadrature Output"));
  bi_decl(bi_1pin_with_name(LB_PIN, "Mouse Button"));
  bi_decl(bi_1pin_with_name(UART_TX_PIN, "UART TX"));
  bi_decl(bi_1pin_with_name(UART_RX_PIN, "UART RX"));
  bi_decl(bi_1pin_with_name(PIO_USB_DP_PIN_DEFAULT, "PIO USB D+"));
  bi_decl(bi_1pin_with_name(PIO_USB_DP_PIN_DEFAULT + 1, "PIO USB D-"));
  bi_decl(bi_1pin_with_name(STATUS_PIN, "Status LED"));
  bi_decl(bi_1pin_with_name(KBD_CLK_PIN, "Keyboard Clock"));
  bi_decl(bi_1pin_with_name(KBD_DATA_PIN, "Keyboard Data"));

  // Initalize the pins
  gpio_init(XA_PIN);
  gpio_init(XB_PIN);
  gpio_init(YA_PIN);
  gpio_init(YB_PIN);
  gpio_init(STATUS_PIN);
  gpio_init(KBD_CLK_PIN);
  gpio_init(KBD_DATA_PIN);
  DEBUG_PRINT(("Pins initalised\r\n"));

  // Set pin directions
  gpio_set_dir(XA_PIN, GPIO_OUT);
  gpio_set_dir(XB_PIN, GPIO_OUT);
  gpio_set_dir(YA_PIN, GPIO_OUT);
  gpio_set_dir(YB_PIN, GPIO_OUT);
  gpio_set_dir(STATUS_PIN, GPIO_OUT);
  gpio_set_dir(KBD_CLK_PIN, GPIO_OUT);
  gpio_set_dir(KBD_DATA_PIN, GPIO_IN);
  DEBUG_PRINT(("Pin directions set\r\n"));

  // Set the pins low
  gpio_put(XA_PIN, 0);
  gpio_put(XB_PIN, 0);
  gpio_put(YA_PIN, 0);
  gpio_put(YB_PIN, 0);
  gpio_put(STATUS_PIN, 0);
  gpio_put(KBD_CLK_PIN, 1);
  gpio_put(KBD_DATA_PIN, 0);
  DEBUG_PRINT(("Pins pulled low\r\n"));
}

//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;
  DEBUG_PRINT(("USB Device Attached\r\n"));

  // Interface protocol (hid_interface_protocol_enum_t)
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  DEBUG_PRINT(("Protocol: %d\r\n", itf_protocol));

  // Receive report from boot mouse only
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (itf_protocol == HID_ITF_PROTOCOL_MOUSE)
  {
    if (tuh_hid_receive_report(dev_addr, instance))
    {
      DEBUG_PRINT(("Mouse Timers Running\r\n"));
    }
  }
  else if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD)
  {
    if (tuh_hid_receive_report(dev_addr, instance))
    {
      DEBUG_PRINT(("Keyboard Ready\r\n"));
    }
  }
  gpio_put(STATUS_PIN, 1); // Turn status LED on
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  (void)dev_addr;
  (void)instance;
  DEBUG_PRINT(("USB Device Removed\r\n"));
  gpio_put(STATUS_PIN, 0); // Turn status LED off
}

static void processMouse(uint8_t dev_addr, hid_mouse_report_t const *report)
{
  // Blink status LED
  // gpio_put(STATUS_PIN, 0);
  (void)dev_addr;
  // Handle mouse buttons
  // Check for left mouse button
  if (report->buttons & (MOUSE_BUTTON_LEFT | MOUSE_BUTTON_RIGHT | MOUSE_BUTTON_MIDDLE))
  {
    gpio_init(LB_PIN);
    gpio_set_dir(LB_PIN, GPIO_OUT);
    gpio_put(LB_PIN, 0);
    DEBUG_PRINT(("Left button press\r\n"));
  }
  else
  {
    gpio_deinit(LB_PIN);
  }

  mouseDistanceX += report->x;
  mouseDistanceY += report->y;
  // Limit the maximum buffered movements
  if (mouseDistanceX > Q_BUFFERLIMIT)
  {
    mouseDistanceX = Q_BUFFERLIMIT;
  }
  else if (mouseDistanceX < -Q_BUFFERLIMIT)
  {
    mouseDistanceX = -Q_BUFFERLIMIT;
  }
  if (mouseDistanceY > Q_BUFFERLIMIT)
  {
    mouseDistanceY = Q_BUFFERLIMIT;
  }
  else if (mouseDistanceY < -Q_BUFFERLIMIT)
  {
    mouseDistanceY = -Q_BUFFERLIMIT;
  }
}

static void processKeyboard(uint8_t dev_addr, hid_keyboard_report_t const *report)
{
  (void)dev_addr;
  // Compare with previous state to detect changes
  // Check for key releases
  for (int i = 0; i < 6; i++) {
    if (prev_keyboard_state.keycode[i] != 0) {
      bool still_pressed = false;
      for (int j = 0; j < 6; j++) {
        if (report->keycode[j] == prev_keyboard_state.keycode[i]) {
          still_pressed = true;
          break;
        }
      }
      if (!still_pressed) {
        // Key has been released
        kbd_enqueue_scancode(prev_keyboard_state.keycode[i], 0); // Release event
      }
    }
  }
  // Check for key presses
  for (int i = 0; i < 6; i++) {
    if (report->keycode[i] != 0) {
      bool already_pressed = false;
      for (int j = 0; j < 6; j++) {
        if (prev_keyboard_state.keycode[j] == report->keycode[i]) {
          already_pressed = true;
          break;
        }
      }
      if (!already_pressed) {
        // Key has been pressed
        kbd_enqueue_scancode(report->keycode[i], 1); // Press event
      }
    }
  }
  //Handle modifier keys, convert into scan codes
  uint8_t modifier_changes = report->modifier ^ prev_keyboard_state.modifier;
  for (int i = 0; i < 8; i++) {
    if (modifier_changes & (1 << i)) {
      // Modifier state has changed
      kbd_enqueue_scancode(0xE0 + i, (report->modifier & (1 << i)) != 0); // Press or release event
    }
  }
  // Save current state as previous state for next comparison
  prev_keyboard_state = *report;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
  switch (itf_protocol)
  {
  case HID_ITF_PROTOCOL_MOUSE:
    processMouse(dev_addr, (hid_mouse_report_t const *)report);
    break;

  case HID_ITF_PROTOCOL_KEYBOARD:
    processKeyboard(dev_addr, (hid_keyboard_report_t const *)report);
    break;
  default:
    break;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    return;
  }
}