#define private public
#define protected public
#include <unity.h>
#include "Arduino.h"
#include "Wire.h"
#include "TLC5955.h"
#include "SPI.h"
#include "atem_client.h"
#include "led_controller.h"
#include "adc_reader.h"
#include "fader_controller.h"
#include "button_manager.h"
#include "MCP23017.h"

// Define mock globals
uint32_t mock_millis_val = 0;
MockSerial Serial;
MockEthernet Ethernet;
TwoWire Wire;
SPIClass SPI(HSPI);
MockW5100 W5100;

AtemClient gAtem(IPAddress(10, 0, 0, 146), 9910, 50991);
TLC5955 gTlc;
MCP23017 gMcp(0x20);
AdcReader gAdc;

LedController gLedController(gTlc, gAtem, gAdc);
ButtonManager gButtonManager(gMcp, gAtem);

void setUp(void) {
  // Reset time (start at 1000 so the first updateLeds call always runs)
  mock_millis_val = 1000;
  gButtonManager.pgmShift_ = false;
  gButtonManager.prvShift_ = false;
  
  // Clear mock TLC5955 grayscale data
  gTlc.set_all(0);
  
  // Reset switcher state
  gAtem.switcherState_ = AtemSwitcherState();
  gAtem.connectionState_ = AtemConnectionState::DISCONNECTED;
  gAtem.initialSyncComplete_ = false;
  gAtem.initialStateSeen_ = false;
  gAtem.markNextConnected_ = false;
  gAtem.initialSyncCommandCount_ = 0;
  
  // Clear Atem Client command queue
  gAtem.commandQueueCount_ = 0;
  gAtem.commandQueueHead_ = 0;
  gAtem.commandQueueTail_ = 0;
}

void tearDown(void) {
}

// Helper to clear command queue completely
void clear_command_queue() {
  gAtem.commandQueueCount_ = 0;
  gAtem.commandQueueHead_ = 0;
  gAtem.commandQueueTail_ = 0;
}

// Helper to get RGB colors of a specific LedId
void get_led_colors(LedId id, uint16_t& r, uint16_t& g, uint16_t& b) {
  const RgbLedMapping* led = gLedController.findRgbLed(id);
  if (led == nullptr) {
    r = g = b = 0xFFFF;
    return;
  }
  r = TLC5955::_grayscale_data[led->redChannel / 48][(led->redChannel % 48) / 3][led->redChannel % 3];
  g = TLC5955::_grayscale_data[led->greenChannel / 48][(led->greenChannel % 48) / 3][led->greenChannel % 3];
  b = TLC5955::_grayscale_data[led->blueChannel / 48][(led->blueChannel % 48) / 3][led->blueChannel % 3];
}

uint16_t get_fader_led_value(size_t index) {
  return TLC5955::_grayscale_data[kFaderLedChannels[index]/48][(kFaderLedChannels[index]%48)/3][kFaderLedChannels[index]%3];
}

size_t append_atem_command(uint8_t* packet, size_t offset, const char* name, const uint8_t* payload, uint16_t payloadLen) {
  const uint16_t cmdLen = 8 + payloadLen;
  packet[offset] = cmdLen >> 8;
  packet[offset + 1] = cmdLen & 0xFF;
  packet[offset + 2] = 0;
  packet[offset + 3] = 0;
  memcpy(packet + offset + 4, name, 4);
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(packet + offset + 8, payload, payloadLen);
  }
  return offset + cmdLen;
}

void finalize_atem_packet(uint8_t* packet, size_t packetLen) {
  packet[0] = 0x08 | ((packetLen >> 8) & 0x07);
  packet[1] = packetLen & 0xFF;
  packet[2] = 0x12;
  packet[3] = 0x34;
  packet[10] = 0;
  packet[11] = 1;
}

// Test 1: AtemClient Profile Auto-detection
void test_profile_autodetection(void) {
  // Test ATEM Mini Pro Profile
  gAtem.setProfileForProductIdentifier((const uint8_t*)"ATEM Mini Pro", 13);
  TEST_ASSERT_EQUAL(AtemModelProfileId::MiniPro, gAtem.profile().id);
  TEST_ASSERT_EQUAL_STRING("ATEM Mini Pro", gAtem.profile().name);

  // Test Constellation HD 1 M/E Profile
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  TEST_ASSERT_EQUAL(AtemModelProfileId::ConstellationHD1ME, gAtem.profile().id);
  TEST_ASSERT_EQUAL_STRING("1 M/E Constellation HD", gAtem.profile().name);

  // Test Unknown/Generic Profile
  gAtem.setProfileForProductIdentifier((const uint8_t*)"Some Unknown ATEM Switcher", 26);
  TEST_ASSERT_EQUAL(AtemModelProfileId::Unknown, gAtem.profile().id);
}

// Test 2: Button Source Mapping for Profiles
void test_button_source_mapping(void) {
  // Test Mini Pro Profile mappings
  gAtem.setProfileForProductIdentifier((const uint8_t*)"ATEM Mini Pro", 13);
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input1, gAtem.sourceForButton(0, false));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input4, gAtem.sourceForButton(3, false));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::None, gAtem.sourceForButton(4, false));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Black, gAtem.sourceForButton(7, false));
  // Shifted sources on Mini Pro are identical to primary
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input1, gAtem.sourceForButton(0, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::None, gAtem.sourceForButton(4, true));

  // Test Constellation HD 1 M/E Profile mappings
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input1, gAtem.sourceForButton(0, false));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input8, gAtem.sourceForButton(7, false));
  
  // Shifted sources on Constellation HD 1 M/E
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input9, gAtem.sourceForButton(0, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input10, gAtem.sourceForButton(1, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::MediaPlayer1, gAtem.sourceForButton(2, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::MediaPlayer2, gAtem.sourceForButton(3, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::ColorGenerator1, gAtem.sourceForButton(4, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::ColorGenerator2, gAtem.sourceForButton(5, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Black, gAtem.sourceForButton(6, true));
  TEST_ASSERT_EQUAL((uint16_t)AtemSource::ColorBars, gAtem.sourceForButton(7, true));
}

// Test 3: Keyer Support Check
void test_keyer_support(void) {
  gAtem.setProfileForProductIdentifier((const uint8_t*)"ATEM Mini Pro", 13);
  TEST_ASSERT_TRUE(gAtem.supportsDownstreamKeyer(0));
  TEST_ASSERT_FALSE(gAtem.supportsDownstreamKeyer(1));

  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  TEST_ASSERT_TRUE(gAtem.supportsDownstreamKeyer(0));
  TEST_ASSERT_FALSE(gAtem.supportsDownstreamKeyer(1));
}

// Test 4: Profile Button / LED Active State check
void test_profile_led_support(void) {
  gAtem.setProfileForProductIdentifier((const uint8_t*)"ATEM Mini Pro", 13);
  
  // Buttons 1-4 and 8 are supported, 5-7 are not
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM1), 1, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM4), 4, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_FALSE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM5), 5, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_FALSE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM6), 6, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_FALSE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM7), 7, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM8), 8, gButtonManager.pgmShift(), gButtonManager.prvShift()));

  // DSK1 supported, DSK2 not
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::DSK1_CUT), 0, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_FALSE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::DSK2_CUT), 0, gButtonManager.pgmShift(), gButtonManager.prvShift()));

  // Constellation HD 1 M/E has all buttons active
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM5), 5, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM6), 6, gButtonManager.pgmShift(), gButtonManager.prvShift()));
  TEST_ASSERT_TRUE(gLedController.isLedSupportedByActiveProfile(*gLedController.findRgbLed(LedId::PGM7), 7, gButtonManager.pgmShift(), gButtonManager.prvShift()));
}

void test_initial_sync_requires_incm_before_ready(void) {
  gAtem.connectionState_ = AtemConnectionState::CONNECTED;
  gAtem.initialSyncComplete_ = false;

  uint8_t packet[64] = {};
  size_t offset = 12;
  const uint8_t programPayload[4] = {0x00, 0x00, 0x00, 0x03};
  offset = append_atem_command(packet, offset, "PrgI", programPayload, sizeof(programPayload));
  finalize_atem_packet(packet, offset);

  gAtem.parseState(packet, offset);

  TEST_ASSERT_EQUAL((uint16_t)AtemSource::Input3, gAtem.switcherState_.programSource);
  TEST_ASSERT_FALSE(gAtem.connected());

  const uint8_t commandPayload[4] = {0x00, 0x00, 0x00, 0x01};
  TEST_ASSERT_FALSE(gAtem.sendCommand("CPgI", commandPayload, sizeof(commandPayload)));
  TEST_ASSERT_EQUAL(0, gAtem.commandQueueCount_);

  uint8_t completePacket[32] = {};
  size_t completeOffset = 12;
  completeOffset = append_atem_command(completePacket, completeOffset, "InCm", nullptr, 0);
  finalize_atem_packet(completePacket, completeOffset);

  gAtem.parseState(completePacket, completeOffset);

  TEST_ASSERT_TRUE(gAtem.connected());
  TEST_ASSERT_TRUE(gAtem.sendCommand("CPgI", commandPayload, sizeof(commandPayload)));
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
}

void test_parse_state_ignores_short_command_payloads(void) {
  gAtem.connectionState_ = AtemConnectionState::CONNECTED;

  uint8_t packet[64] = {};
  size_t offset = 12;
  const uint8_t shortProgramPayload[2] = {0x00, 0x00};
  offset = append_atem_command(packet, offset, "PrgI", shortProgramPayload, sizeof(shortProgramPayload));
  offset = append_atem_command(packet, offset, "InCm", nullptr, 0);
  finalize_atem_packet(packet, offset);

  gAtem.parseState(packet, offset);

  TEST_ASSERT_EQUAL(kUnknownAtemSource, gAtem.switcherState_.programSource);
  TEST_ASSERT_TRUE(gAtem.connected());
}

void test_fader_sends_absolute_position_and_uses_atem_return(void) {
  FaderController fader(gAdc, gAtem);

  gAtem.connectionState_ = AtemConnectionState::CONNECTED;
  gAtem.initialSyncComplete_ = true;
  gAtem.switcherState_.virtualFaderAtTop = false;

  TEST_ASSERT_EQUAL(0, fader.mapAdcToAtemPosition(0));
  TEST_ASSERT_EQUAL(kAtemTransitionPositionMax, fader.mapAdcToAtemPosition(4095));

  gAdc.channels_[0] = 4095;
  fader.update();

  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL_INT8('C', gAtem.commandQueue_[0].name[0]);
  TEST_ASSERT_EQUAL_INT8('T', gAtem.commandQueue_[0].name[1]);
  TEST_ASSERT_EQUAL_INT8('P', gAtem.commandQueue_[0].name[2]);
  TEST_ASSERT_EQUAL_INT8('s', gAtem.commandQueue_[0].name[3]);
  TEST_ASSERT_EQUAL(0x27, gAtem.commandQueue_[0].payload[2]);
  TEST_ASSERT_EQUAL(0x10, gAtem.commandQueue_[0].payload[3]);

  uint8_t packet[32] = {};
  size_t offset = 12;
  const uint8_t transitionPayload[6] = {0x00, 0x01, 0x00, 0x00, 0x13, 0x88};
  offset = append_atem_command(packet, offset, "TrPs", transitionPayload, sizeof(transitionPayload));
  finalize_atem_packet(packet, offset);

  gAtem.parseState(packet, offset);

  TEST_ASSERT_EQUAL(5000, gAtem.switcherState_.transitionPosition);
  TEST_ASSERT_EQUAL(5000, gAtem.switcherState_.faderLedPosition);
  TEST_ASSERT_TRUE(gAtem.switcherState_.transitionInProgress);
  TEST_ASSERT_TRUE(gAtem.switcherState_.faderTransitionActive);
}

// Test 5: LED Color Logic (Program and Preview)
void test_led_color_logic_pgm_prv(void) {
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  uint16_t r, g, b;

  // Case 1: Program Input 1 is active (primary)
  gAtem.switcherState_.programSource = (uint16_t)AtemSource::Input1;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PGM1, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Solid Red
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  get_led_colors(LedId::PGM2, r, g, b);
  TEST_ASSERT_EQUAL(kDimBrightness, r); // Dim white/blue-ish
  TEST_ASSERT_EQUAL(kDimBrightness, g);
  TEST_ASSERT_EQUAL(kDimBrightness, b);

  // Case 2: Program Input 10 is active (shifted for button 2)
  gAtem.switcherState_.programSource = (uint16_t)AtemSource::Input10;
  
  // Set time so blinkState is true
  mock_millis_val = 2000; // (2000 / 250) % 2 == 8 % 2 == 0 -> true
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PGM2, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  // Set time so blinkState is false
  mock_millis_val = 2250; // (2250 / 250) % 2 == 9 % 2 == 1 -> false
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PGM2, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  // Case 3: Preview Input 1 is active (primary)
  gAtem.switcherState_.previewSource = (uint16_t)AtemSource::Input1;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PRV1, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(kLedBrightness, g); // Solid Green
  TEST_ASSERT_EQUAL(0, b);

  // Case 4: Preview Input 10 is active (shifted for button 2)
  gAtem.switcherState_.previewSource = (uint16_t)AtemSource::Input10;
  mock_millis_val = 3000; // (3000 / 250) % 2 == 12 % 2 == 0 -> true
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PRV2, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(kLedBrightness, g);
  TEST_ASSERT_EQUAL(0, b);

  mock_millis_val = 3250; // (3250 / 250) % 2 == 13 % 2 == 1 -> false
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PRV2, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);
}

// Test 6: Shift, Transition, and Keyer LED Colors
void test_transition_keyer_leds(void) {
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  uint16_t r, g, b;

  // Case 1: Shift buttons held
  gButtonManager.pgmShift_ = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PGM_SHIFT, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Yellow
  TEST_ASSERT_EQUAL(kLedBrightness, g);
  TEST_ASSERT_EQUAL(0, b);

  gButtonManager.pgmShift_ = false;
  gButtonManager.prvShift_ = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::PRV_SHIFT, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Yellow
  TEST_ASSERT_EQUAL(kLedBrightness, g);
  TEST_ASSERT_EQUAL(0, b);

  gButtonManager.prvShift_ = false;

  // Case 2: BKGD & KEY1 TIE active (Transition Selection)
  gAtem.switcherState_.nextTrBkgd = true;
  gAtem.switcherState_.nextTrKey1 = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::BKGD, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Yellow
  TEST_ASSERT_EQUAL(kLedBrightness, g);
  get_led_colors(LedId::KEY1_TIE, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Yellow
  TEST_ASSERT_EQUAL(kLedBrightness, g);

  // Case 3: Key 1 On Air
  gAtem.switcherState_.key1OnAir = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::KEY1_CUT, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Red
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  // Case 4: DSK 1 On Air and Tie
  gAtem.switcherState_.dskOnAir[0] = true;
  gAtem.switcherState_.dskTie[0] = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::DSK1_CUT, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Red
  get_led_colors(LedId::DSK1_TIE, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Yellow
  TEST_ASSERT_EQUAL(kLedBrightness, g);

  // Case 5: DSK 1 Transitioning (AUTO button blinks red)
  gAtem.switcherState_.dskTransitioning[0] = true;
  mock_millis_val = 4000; // blinkState = true
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::DSK1_AUTO, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r);
  TEST_ASSERT_EQUAL(0, g);

  mock_millis_val = 4250; // blinkState = false
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::DSK1_AUTO, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
}

// Test 7: Fade to Black (FTB) states
void test_fade_to_black_led(void) {
  uint16_t r, g, b;

  // Case 1: Disconnected (blinking Blue)
  gAtem.connectionState_ = AtemConnectionState::DISCONNECTED;
  mock_millis_val = 5000; // blinkState = true
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::FTB, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(kLedBrightness, b); // Blue

  mock_millis_val = 5250; // blinkState = false
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::FTB, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  // Case 2: Connected & FTB Active (blinking Red)
  gAtem.connectionState_ = AtemConnectionState::CONNECTED;
  gAtem.initialSyncComplete_ = true;
  gAtem.switcherState_.ftbActive = true;
  mock_millis_val = 6000; // blinkState = true
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::FTB, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Red
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  mock_millis_val = 6250; // blinkState = false
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::FTB, r, g, b);
  TEST_ASSERT_EQUAL(0, r);
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);

  // Case 3: Connected & FTB Done (solid Red)
  gAtem.switcherState_.ftbActive = false;
  gAtem.switcherState_.ftbDone = true;
  mock_millis_val += 100;
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  get_led_colors(LedId::FTB, r, g, b);
  TEST_ASSERT_EQUAL(kLedBrightness, r); // Solid Red
  TEST_ASSERT_EQUAL(0, g);
  TEST_ASSERT_EQUAL(0, b);
}

// Test 8: Fader LED progress bar filling
void test_fader_led_positions(void) {
  // Idle: ADC 0 shows the physical fader at the top endpoint.
  gAdc.channels_[0] = 0;
  gLedController.updateFaderLeds();
  TEST_ASSERT_EQUAL(kFaderLedBrightness, get_fader_led_value(0));
  for (size_t i = 1; i < kFaderLedCount; ++i) {
    TEST_ASSERT_EQUAL(0, get_fader_led_value(i));
  }

  // Active transition fills from LED 1 toward LED 16 using ATEM TrPs.
  gTlc.set_all(0);
  gAtem.switcherState_.transitionInProgress = true;
  gAtem.switcherState_.faderTransitionActive = true;
  gAtem.switcherState_.transitionStartedAtTop = true;
  gAtem.switcherState_.faderLedPosition = 5000;
  gAtem.switcherState_.transitionPosition = 5000;
  gLedController.updateFaderLeds();
  for (size_t i = 0; i < kFaderLedCount; ++i) {
    uint16_t expected = (i < 8) ? kFaderLedBrightness : 0;
    TEST_ASSERT_EQUAL(expected, get_fader_led_value(i));
  }

  // A larger TrPs value lights more sequential LEDs.
  gTlc.set_all(0);
  gAtem.switcherState_.faderLedPosition = 7500;
  gAtem.switcherState_.transitionPosition = 7500;
  gLedController.updateFaderLeds();
  for (size_t i = 0; i < kFaderLedCount; ++i) {
    uint16_t expected = (i < 12) ? kFaderLedBrightness : 0;
    TEST_ASSERT_EQUAL(expected, get_fader_led_value(i));
  }

  // Moving the physical fader from ADC 4095 toward 0 starts from LED 16.
  gTlc.set_all(0);
  gAtem.switcherState_.transitionStartedAtTop = false;
  gAtem.switcherState_.faderLedPosition = 5000;
  gAtem.switcherState_.transitionPosition = 5000;
  gLedController.updateFaderLeds();
  for (size_t i = 0; i < kFaderLedCount; ++i) {
    uint16_t expected = (i >= 8) ? kFaderLedBrightness : 0;
    TEST_ASSERT_EQUAL(expected, get_fader_led_value(i));
  }

  gTlc.set_all(0);
  gAtem.switcherState_.faderLedPosition = 7500;
  gAtem.switcherState_.transitionPosition = 7500;
  gLedController.updateFaderLeds();
  for (size_t i = 0; i < kFaderLedCount; ++i) {
    uint16_t expected = (i >= 4) ? kFaderLedBrightness : 0;
    TEST_ASSERT_EQUAL(expected, get_fader_led_value(i));
  }

  // Full progress lights the whole bar.
  gTlc.set_all(0);
  gAtem.switcherState_.faderLedPosition = kAtemTransitionPositionMax;
  gAtem.switcherState_.transitionPosition = kAtemTransitionPositionMax;
  gLedController.updateFaderLeds();
  for (size_t i = 0; i < kFaderLedCount; ++i) {
    TEST_ASSERT_EQUAL(kFaderLedBrightness, get_fader_led_value(i));
  }
}


// Test 9: Button Press dispatching and commands enqueuing
void test_button_dispatching(void) {
  gAtem.setProfileForProductIdentifier((const uint8_t*)"1 M/E Constellation HD", 22);
  gAtem.connectionState_ = AtemConnectionState::CONNECTED;
  gAtem.initialSyncComplete_ = true;

  // Press PGM1
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(50, 45), 50, 45);
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL_INT8('C', gAtem.commandQueue_[0].name[0]);
  TEST_ASSERT_EQUAL_INT8('P', gAtem.commandQueue_[0].name[1]);
  TEST_ASSERT_EQUAL_INT8('g', gAtem.commandQueue_[0].name[2]);
  TEST_ASSERT_EQUAL_INT8('I', gAtem.commandQueue_[0].name[3]);
  TEST_ASSERT_EQUAL(4, gAtem.commandQueue_[0].payloadLen);
  TEST_ASSERT_EQUAL(0x00, gAtem.commandQueue_[0].payload[0]);
  TEST_ASSERT_EQUAL(0x00, gAtem.commandQueue_[0].payload[1]);
  TEST_ASSERT_EQUAL(0x00, gAtem.commandQueue_[0].payload[2]);
  TEST_ASSERT_EQUAL(0x01, gAtem.commandQueue_[0].payload[3]); // Camera 1 source

  // Hold PGM Shift and press PGM1 again (should map to Input 9)
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(50, 37), 50, 37); // Press Shift
  TEST_ASSERT_TRUE(gButtonManager.pgmShift_);
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(50, 45), 50, 45); // Press PGM1
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL(0x00, gAtem.commandQueue_[0].payload[2]);
  TEST_ASSERT_EQUAL(0x09, gAtem.commandQueue_[0].payload[3]); // Input 9 source
  gButtonManager.handleButtonRelease(gButtonManager.findButtonMapping(50, 37), 50, 37); // Release Shift
  TEST_ASSERT_FALSE(gButtonManager.pgmShift_);

  // Press CUT
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(48, 45), 48, 45);
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL_INT8('D', gAtem.commandQueue_[0].name[0]);
  TEST_ASSERT_EQUAL_INT8('C', gAtem.commandQueue_[0].name[1]);
  TEST_ASSERT_EQUAL_INT8('u', gAtem.commandQueue_[0].name[2]);
  TEST_ASSERT_EQUAL_INT8('t', gAtem.commandQueue_[0].name[3]);

  // Press AUTO
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(47, 45), 47, 45);
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL_INT8('D', gAtem.commandQueue_[0].name[0]);
  TEST_ASSERT_EQUAL_INT8('A', gAtem.commandQueue_[0].name[1]);
  TEST_ASSERT_EQUAL_INT8('u', gAtem.commandQueue_[0].name[2]);
  TEST_ASSERT_EQUAL_INT8('t', gAtem.commandQueue_[0].name[3]);

  // Press KEY1 CUT (should toggle Key 1 state on air)
  gAtem.switcherState_.key1OnAir = false;
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(47, 44), 47, 44);
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL(0x01, gAtem.commandQueue_[0].payload[2]); // toggle to true

  // Press BKGD (should toggle transition Bkgd state)
  gAtem.switcherState_.nextTrBkgd = true;
  gAtem.switcherState_.nextTrKey1 = false;
  clear_command_queue();
  gButtonManager.handleButtonPress(gButtonManager.findButtonMapping(48, 43), 48, 43);
  TEST_ASSERT_EQUAL(1, gAtem.commandQueueCount_);
  TEST_ASSERT_EQUAL_INT8('C', gAtem.commandQueue_[0].name[0]);
  TEST_ASSERT_EQUAL_INT8('T', gAtem.commandQueue_[0].name[1]);
  TEST_ASSERT_EQUAL_INT8('T', gAtem.commandQueue_[0].name[2]);
  TEST_ASSERT_EQUAL_INT8('p', gAtem.commandQueue_[0].name[3]);
  TEST_ASSERT_EQUAL(0x00, gAtem.commandQueue_[0].payload[3]); // Bkgd toggles to false, key1 remains false -> 0x00
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_profile_autodetection);
  RUN_TEST(test_button_source_mapping);
  RUN_TEST(test_keyer_support);
  RUN_TEST(test_profile_led_support);
  RUN_TEST(test_initial_sync_requires_incm_before_ready);
  RUN_TEST(test_parse_state_ignores_short_command_payloads);
  RUN_TEST(test_fader_sends_absolute_position_and_uses_atem_return);
  RUN_TEST(test_led_color_logic_pgm_prv);
  RUN_TEST(test_transition_keyer_leds);
  RUN_TEST(test_fade_to_black_led);
  RUN_TEST(test_fader_led_positions);
  RUN_TEST(test_button_dispatching);
  return UNITY_END();
}
