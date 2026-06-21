/**
 * File: emul.c
 * Author: Diego Parrilla Santamaría
 * Date: February 2025, February 2026
 * Copyright: 2025-2026 - GOODDATA LABS
 * Description: Fast-serial emulation — USB CDC ↔ Atari ST AUX: bridge
 */

#include "emul.h"

#include <stdint.h>
#include <stdio.h>

#include "aconfig.h"
#include "chandler.h"
#include "commemul.h"
#include "constants.h"
#include "debug.h"
#include "display.h"
#include "ff.h"
#include "gconfig.h"
#include "memfunc.h"
#include "network.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "reset.h"
#include "romemul.h"
#include "sdcard.h"
#include "select.h"
#include "target_firmware.h"
#include "term.h"

// Poll the main loop at most every 1 ms so the chandler stays responsive
// during send_sync round-trips from the ST side.
#define SLEEP_LOOP_MS 1

enum {
  APP_MODE_SETUP = 255
};

// RP→ST ring buffer state (module-level so serial_command_cb can access it)
static uint8_t  *serial_rx_ring     = NULL;
static uint32_t  serial_rx_write_ptr = 0;
static uint32_t  serial_rx_read_ack  = 0;  // last read ptr ACK'd by ST

// Command handlers
static void cmdMenu(const char *arg);
static void cmdClear(const char *arg);
static void cmdExit(const char *arg);
static void cmdFirmware(const char *arg);
static void cmdHelp(const char *arg);
static void cmdBooster(const char *arg);
static void cmdSettings(const char *arg);
static void cmdPrint(const char *arg);
static void cmdSave(const char *arg);
static void cmdErase(const char *arg);
static void cmdGet(const char *arg);
static void cmdPutInt(const char *arg);
static void cmdPutBool(const char *arg);
static void cmdPutString(const char *arg);

static const Command commands[] = {
    {"m", cmdMenu},
    {"h", cmdHelp},
    {"e", cmdExit},
    {"f", cmdFirmware},
    {"x", cmdBooster},
    {"?", cmdHelp},
    {"s", cmdSettings},
    {"settings", cmdSettings},
    {"print", cmdPrint},
    {"save", cmdSave},
    {"erase", cmdErase},
    {"get", cmdGet},
    {"put_int", cmdPutInt},
    {"put_bool", cmdPutBool},
    {"put_str", cmdPutString},
};
static const size_t numCommands = sizeof(commands) / sizeof(commands[0]);

static bool keepActive     = true;
static bool menuScreenActive = false;
static absolute_time_t menuRefreshTime;

static void __not_in_flash_func(emul_pollTick)(void) {
  chandler_loop();
  term_loop();
}

#define MENU_REFRESH_TIME_MS 1000

static bool resetDeviceAtBoot = true;

static void showTitle() {
  term_printString(
      "\x1B"
      "E"
      "FastSerial - " RELEASE_VERSION "\n");
}

static void menu(void) {
  menuScreenActive = true;
  showTitle();
  term_printString("\n\n");
  term_printString("[S]ettings     | [F]irmware launch\n");
  term_printString("[E]xit desktop | [X] Back to Booster\n\n");
  term_printNetworkInfo();
  term_printString("\n");
  term_printString("Select an option: ");
  term_markMenuPromptCursor();
  menuRefreshTime = make_timeout_time_ms(MENU_REFRESH_TIME_MS);
}

void cmdMenu(const char *arg)     { menu(); }

void cmdHelp(const char *arg) {
  menuScreenActive = false;
  term_printString("Available commands:\n");
  term_printString(" General:\n");
  term_printString("  f       - Launch serial bridge on the Atari ST\n");
  term_printString("  help    - Show available commands\n");
}

void cmdClear(const char *arg) {
  menuScreenActive = false;
  term_clearScreen();
}

void cmdExit(const char *arg) {
  menuScreenActive = false;
  term_printString("Exiting terminal...\n");
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_CONTINUE);
}

void cmdFirmware(const char *arg) {
  menuScreenActive = false;
  term_printString("Launching serial bridge on the Atari ST...\n");
  // Triggers the m68k's check_commands to branch to rom_function → USERFW.
  // userfw.s installs the BIOS trap-13 hook and stores the ST RAM buffer
  // pointer via CMD_SET_SHARED_VAR before returning to TOS to boot GEM.
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_START);
}

void cmdBooster(const char *arg) {
  menuScreenActive = false;
  term_printString("Launching Booster app...\n");
  term_printString("The computer will boot shortly...\n\n");
  term_printString("If it doesn't boot, power it on and off.\n");
  resetDeviceAtBoot = false;
  keepActive = false;
}

void cmdSettings(const char *arg) { menuScreenActive = false; term_cmdSettings(arg); }
void cmdPrint(const char *arg)    { menuScreenActive = false; term_cmdPrint(arg); }
void cmdSave(const char *arg)     { menuScreenActive = false; term_cmdSave(arg); }
void cmdErase(const char *arg)    { menuScreenActive = false; term_cmdErase(arg); }
void cmdGet(const char *arg)      { menuScreenActive = false; term_cmdGet(arg); }
void cmdPutInt(const char *arg)   { menuScreenActive = false; term_cmdPutInt(arg); }
void cmdPutBool(const char *arg)  { menuScreenActive = false; term_cmdPutBool(arg); }
void cmdPutString(const char *arg){ menuScreenActive = false; term_cmdPutString(arg); }

static bool getKeepActive()   { return keepActive; }
static bool getResetDevice()  { return resetDeviceAtBoot; }

// ---------------------------------------------------------------------------
// Serial bridge: chandler callback
// Handles three command IDs from the ST:
//   1 (CMD_SET_SHARED_VAR, payload_size==12): write to a shared variable.
//       The ST sends this for detect_hw, get_tos_version, and to store the
//       BIOS-hook ST-RAM buffer pointer in CHANDLER_SERIAL_ST_BUFPTR (idx 2).
//   APP_SERIAL_TX (2): one byte from ST, forward to USB CDC.
//   APP_SERIAL_RX_ACK (3): ST consumed N bytes, update ring-buffer flow ctrl.
// ---------------------------------------------------------------------------
static void __not_in_flash_func(serial_command_cb)(
    TransmissionProtocol *protocol, uint16_t *payloadPtr) {

  uint32_t shared_base = (uint32_t)&__rom_in_ram_start__;

  switch (protocol->command_id) {
    case 1: {
      // CMD_SET_SHARED_VAR — payload_size 12 (4 token + 4 index + 4 value).
      // payload_size 8 is APP_TERMINAL_KEYSTROKE; ignore those here.
      if (protocol->payload_size != 12) break;
      uint32_t index = TPROTO_GET_PAYLOAD_PARAM32(payloadPtr);
      TPROTO_NEXT32_PAYLOAD_PTR(payloadPtr);
      uint32_t value = TPROTO_GET_PAYLOAD_PARAM32(payloadPtr);
      SET_SHARED_VAR(index, value, shared_base, CHANDLER_SHARED_VARIABLES_OFFSET);
      break;
    }

    case APP_SERIAL_TX: {
      // ST sends one byte to PC. Low byte of the payload word is the char.
      uint16_t charWord = TPROTO_GET_PAYLOAD_PARAM16(payloadPtr);
      putchar_raw((int)(charWord & 0xFF));
      fflush(stdout);
      break;
    }

    case APP_SERIAL_RX_ACK: {
      // ST reports its new read pointer; use it for ring-buffer flow control.
      uint32_t new_read_ptr = TPROTO_GET_PAYLOAD_PARAM32(payloadPtr);
      serial_rx_read_ack = new_read_ptr;
      DPRINTF("SERIAL_RX_ACK: read_ack=%lu write=%lu\n",
              (unsigned long)serial_rx_read_ack,
              (unsigned long)serial_rx_write_ptr);
      break;
    }

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Fill the RP→ST ring buffer from USB CDC.
// Called every main-loop iteration; stops when the ring is full or USB has
// no more data.  Updates CHANDLER_SERIAL_RX_WR_PTR once per call.
// ---------------------------------------------------------------------------
static void __not_in_flash_func(serial_fill_ring)(uint32_t shared_base) {
  bool updated = false;
  while (serial_rx_write_ptr - serial_rx_read_ack <
         (uint32_t)(CHANDLER_SERIAL_RX_RING_SIZE - 1)) {
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) break;
    uint32_t idx = serial_rx_write_ptr & (CHANDLER_SERIAL_RX_RING_SIZE - 1);
    serial_rx_ring[idx] = (uint8_t)ch;
    serial_rx_write_ptr++;
    updated = true;
  }
  if (updated) {
    SET_SHARED_VAR(CHANDLER_SERIAL_RX_WR_PTR, serial_rx_write_ptr,
                   shared_base, CHANDLER_SHARED_VARIABLES_OFFSET);
  }
}

static void preinit() {
  term_init();
  term_clearScreen();
  showTitle();
  term_printString("\n\n");
  term_printString("Configuring network... please wait...\n");
  display_refresh();
}

void failure(const char *message) {
  term_init();
  term_clearScreen();
  showTitle();
  term_printString("\n\n");
  term_printString(message);
  display_refresh();
}

static void init(void) {
  term_setCommands(commands, numCommands);
  term_clearScreen();
  menu();
  display_refresh();
}

void emul_start() {
  uint32_t shared_base = (uint32_t)&__rom_in_ram_start__;

  // Point the ring buffer into the APP_FREE area of shared RAM.
  // The ST sees this region at $FA2300; the RP writes it directly here.
  serial_rx_ring = (uint8_t *)(shared_base + CHANDLER_SERIAL_RX_RING_OFFSET);

  // Initialise USB CDC (safe to call even if stdio_init_all already ran it).
  stdio_usb_init();

  // 1. Check configuration mode
  SettingsConfigEntry *appMode =
      settings_find_entry(aconfig_getContext(), ACONFIG_PARAM_MODE);
  int appModeValue = APP_MODE_SETUP;
  if (appMode != NULL) {
    appModeValue = atoi(appMode->value);
    DPRINTF("Start emulation in mode: %i\n", appModeValue);
  }

  // 2. Copy the m68k firmware to shared RAM and start the cartridge bus engine
  COPY_FIRMWARE_TO_RAM((uint16_t *)target_firmware, target_firmware_length);

  if (init_romemul(false) < 0) {
    panic("init_romemul failed");
  }
  if (commemul_init() < 0) {
    panic("commemul_init failed");
  }

  // 3. Register command callbacks.
  // serial_command_cb handles CMD_SET_SHARED_VAR (writes shared variables),
  // APP_SERIAL_TX and APP_SERIAL_RX_ACK.  term_command_cb handles terminal
  // keystrokes; it also buffers the other commands harmlessly.
  chandler_init();
  chandler_addCB(serial_command_cb);
  chandler_addCB(term_command_cb);

  // 4. Display
  display_setupU8g2();

  // 5. SD card
  FATFS fsys;
  SettingsConfigEntry *folder =
      settings_find_entry(aconfig_getContext(), ACONFIG_PARAM_FOLDER);
  char *folderName = "/fastserial";
  if (folder != NULL) {
    DPRINTF("FOLDER: %s\n", folder->value);
    folderName = folder->value;
  }
  int sdcardErr = sdcard_initFilesystem(&fsys, folderName);
  if (sdcardErr != SDCARD_INIT_OK) {
    DPRINTF("SD card unavailable (%i). Continuing.\n", sdcardErr);
  }

  display_setupU8g2();
  preinit();

  // 6. Network
  SettingsConfigEntry *wifiMode =
      settings_find_entry(gconfig_getContext(), PARAM_WIFI_MODE);
  if (wifiMode != NULL) {
    wifi_mode_t wifiModeValue = (wifi_mode_t)atoi(wifiMode->value);
    if (wifiModeValue != WIFI_MODE_AP) {
      int err = network_wifiInit(WIFI_MODE_STA);
      if (err == 0) {
        network_setPollingCallback(emul_pollTick);
        int attempt = 0, maxAttempts = 3;
        err = NETWORK_WIFI_STA_CONN_ERR_TIMEOUT;
        while (attempt < maxAttempts && err == NETWORK_WIFI_STA_CONN_ERR_TIMEOUT) {
          err = network_wifiStaConnect();
          attempt++;
        }
        network_setPollingCallback(NULL);
      }
    }
  }

  // 7. SELECT button
  select_configure();

  // 8. Terminal
  init();

#ifdef BLINK_H
  blink_on();
#endif

  // 9. Main loop
  DPRINTF("Entering serial bridge main loop\n");
  while (getKeepActive()) {
#if PICO_CYW43_ARCH_POLL
    network_safePoll();
    cyw43_arch_wait_for_work_until(make_timeout_time_ms(SLEEP_LOOP_MS));
#else
    sleep_ms(SLEEP_LOOP_MS);
#endif
    // Drain ROM3 ring → dispatch to callbacks (serial_command_cb + term_command_cb)
    chandler_loop();

    // Terminal foreground
    term_loop();

    // Fill the RP→ST ring buffer with any bytes waiting on the USB port
    serial_fill_ring(shared_base);

    if (menuScreenActive) {
      char *input = term_getInputBuffer();
      bool hasPendingInput = (input != NULL) && (input[0] != '\0');
      if (!hasPendingInput &&
          absolute_time_diff_us(get_absolute_time(), menuRefreshTime) <= 0) {
        term_refreshMenuLiveInfo();
        menuRefreshTime = make_timeout_time_ms(MENU_REFRESH_TIME_MS);
      }
    }
  }

  // 10. Reset or jump to booster
  sleep_ms(SLEEP_LOOP_MS);
  SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_RESET);
  sleep_ms(SLEEP_LOOP_MS);
  if (getResetDevice()) {
    reset_device();
  } else {
    settings_put_integer(aconfig_getContext(), ACONFIG_PARAM_MODE, APP_MODE_SETUP);
    settings_save(aconfig_getContext(), true);
    DPRINTF("Jumping to booster\n");
    reset_jump_to_booster();
  }
}
