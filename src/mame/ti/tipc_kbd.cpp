// license:BSD-3-Clause
// copyright-holders:TODO (author name)
/***************************************************************************

    Texas Instruments Professional Computer keyboard (HLE)

    The real keyboard has its own microcontroller (not dumped).  This is a
    high-level emulation of its serial protocol as described in the TIPC
    Technical Reference (1984) and as exercised by the V1.24 system ROM:

    - async serial, 8 data bits, even parity, 1 stop bit (p. 4-58)
    - keyboard -> system at 2440 baud, system -> keyboard at 305 baud
      (p. 2-3, 2-4, 4-58)
    - each key press sends a key-position byte 01H-68H (key number from
      Table 4-10, p. 4-49..4-51); bit 7 is set on repeat-action
      transmissions (p. 4-58).  No codes are sent on key release.
    - if the mode keys changed since the last transmission, the key byte is
      preceded by a mode byte: bits 3-6 = 1, bit 0 = CTRL, bit 1 = ALT,
      bit 2 = SHIFT, bit 7 = CAPS LOCK (Figure 4-7, p. 4-59).  The mode
      byte is never sent alone.
    - repeat-action starts after 0.5 s and repeats at 15/s, only for the
      most recently pressed key (p. 2-6)
    - commands and responses per Table 2-1 (p. 2-5); 70H = self-test OK

    TODO:
    - response time after a command is not documented (5 ms assumed)
    - "return version" (08H) response bytes are OCR-ambiguous in Table 2-1
      ("10,13", read here as 70H,73H)
    - keyclick (05H/06H) needs a hardware modification and is ignored
    - no parity/framing error checking on received commands

***************************************************************************/

#include "emu.h"
#include "tipc_kbd.h"

#include "machine/keyboard.ipp"


DEFINE_DEVICE_TYPE(TIPC_KEYBOARD, tipc_keyboard_device, "tipc_kbd", "TI Professional Computer Keyboard")

namespace {

INPUT_PORTS_START( tipc_keyboard )
	PORT_START("ROW0")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_UNUSED) // 00
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5) PORT_CHAR(UCHAR_MAMEKEY(F5)) // 01 (key #1)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F6") PORT_CODE(KEYCODE_F6) PORT_CHAR(UCHAR_MAMEKEY(F6)) // 02 (key #2)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F7") PORT_CODE(KEYCODE_F7) PORT_CHAR(UCHAR_MAMEKEY(F7)) // 03 (key #3)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F8") PORT_CODE(KEYCODE_F8) PORT_CHAR(UCHAR_MAMEKEY(F8)) // 04 (key #4)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F9") PORT_CODE(KEYCODE_F9) PORT_CHAR(UCHAR_MAMEKEY(F9)) // 05 (key #5)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F10") PORT_CODE(KEYCODE_F10) PORT_CHAR(UCHAR_MAMEKEY(F10)) // 06 (key #6)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F11") PORT_CODE(KEYCODE_F11) PORT_CHAR(UCHAR_MAMEKEY(F11)) // 07 (key #7)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F12") PORT_CODE(KEYCODE_F12) PORT_CHAR(UCHAR_MAMEKEY(F12)) // 08 (key #8)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("1  !") PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') // 09 (key #9)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("2  @") PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@') // 0A (key #10)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("3  #") PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') // 0B (key #11)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("4  $") PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') // 0C (key #12)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("5  %") PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') // 0D (key #13)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("6  ^") PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') // 0E (key #14)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("7  &") PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&') // 0F (key #15)

	PORT_START("ROW1")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("8  *") PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('*') // 10 (key #16)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("9  (") PORT_CODE(KEYCODE_9) PORT_CHAR('9') PORT_CHAR('(') // 11 (key #17)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("0  )") PORT_CODE(KEYCODE_0) PORT_CHAR('0') PORT_CHAR(')') // 12 (key #18)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("-  _") PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_') // 13 (key #19)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("=  +") PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+') // 14 (key #20)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Backspace") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8) // 15 (key #21)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("`  ~") PORT_CODE(KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('~') // 16 (key #22)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad =") PORT_CODE(KEYCODE_SLASH_PAD) PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)) // 17 (key #23)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad +") PORT_CODE(KEYCODE_PLUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD)) // 18 (key #24)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Space") PORT_CODE(KEYCODE_ASTERISK) PORT_CHAR(UCHAR_MAMEKEY(ASTERISK)) // 19 (key #25)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Tab") PORT_CODE(KEYCODE_NUMLOCK) PORT_CHAR(UCHAR_MAMEKEY(NUMLOCK)) // 1A (key #26)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 1") PORT_CODE(KEYCODE_1_PAD) PORT_CHAR(UCHAR_MAMEKEY(1_PAD)) // 1B (key #27)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_UNUSED) // 1C
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 0") PORT_CODE(KEYCODE_0_PAD) PORT_CHAR(UCHAR_MAMEKEY(0_PAD)) // 1D (key #29)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad Enter") PORT_CODE(KEYCODE_ENTER_PAD) PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) // 1E (key #30)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 4") PORT_CODE(KEYCODE_4_PAD) PORT_CHAR(UCHAR_MAMEKEY(4_PAD)) // 1F (key #31)

	PORT_START("ROW2")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 5") PORT_CODE(KEYCODE_5_PAD) PORT_CHAR(UCHAR_MAMEKEY(5_PAD)) // 20 (key #32)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 9") PORT_CODE(KEYCODE_9_PAD) PORT_CHAR(UCHAR_MAMEKEY(9_PAD)) // 21 (key #33)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad -") PORT_CODE(KEYCODE_MINUS_PAD) PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)) // 22 (key #34)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 2") PORT_CODE(KEYCODE_2_PAD) PORT_CHAR(UCHAR_MAMEKEY(2_PAD)) // 23 (key #35)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_UNUSED) // 24
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_UNUSED) // 25
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_UNUSED) // 26
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 7") PORT_CODE(KEYCODE_7_PAD) PORT_CHAR(UCHAR_MAMEKEY(7_PAD)) // 27 (key #39)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 8") PORT_CODE(KEYCODE_8_PAD) PORT_CHAR(UCHAR_MAMEKEY(8_PAD)) // 28 (key #40)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 6") PORT_CODE(KEYCODE_6_PAD) PORT_CHAR(UCHAR_MAMEKEY(6_PAD)) // 29 (key #41)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad ,") // 2A (key #42)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad 3") PORT_CODE(KEYCODE_3_PAD) PORT_CHAR(UCHAR_MAMEKEY(3_PAD)) // 2B (key #43)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Keypad .") PORT_CODE(KEYCODE_DEL_PAD) PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD)) // 2C (key #44)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_UNUSED) // 2D
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)) // 2E (key #46)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Ins") PORT_CODE(KEYCODE_INSERT) PORT_CHAR(UCHAR_MAMEKEY(INSERT)) // 2F (key #47)

	PORT_START("ROW3")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Del") PORT_CODE(KEYCODE_DEL) PORT_CHAR(UCHAR_MAMEKEY(DEL)) // 30 (key #48)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tab") PORT_CODE(KEYCODE_TAB) PORT_CHAR(9) // 31 (key #49)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Q") PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q') // 32 (key #50)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("W") PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W') // 33 (key #51)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E') // 34 (key #52)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R') // 35 (key #53)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("T") PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T') // 36 (key #54)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Y") PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y') // 37 (key #55)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("U") PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U') // 38 (key #56)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("I") PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I') // 39 (key #57)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("O") PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O') // 3A (key #58)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P') // 3B (key #59)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("[  {") PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{') // 3C (key #60)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("]  }") PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(']') PORT_CHAR('}') // 3D (key #61)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Line Feed") PORT_CODE(KEYCODE_END) PORT_CHAR(10) // 3E (key #62)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_UNUSED) // 3F

	PORT_START("ROW4")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP)) // 40 (key #64)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Esc") PORT_CODE(KEYCODE_ESC) PORT_CHAR(27) // 41 (key #65)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A') // 42 (key #66)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S') // 43 (key #67)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D') // 44 (key #68)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F') // 45 (key #69)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G') // 46 (key #70)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("H") PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H') // 47 (key #71)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("J") PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J') // 48 (key #72)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("K") PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K') // 49 (key #73)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L') // 4A (key #74)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(";  :") PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':') // 4B (key #75)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("'  \"") PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"') // 4C (key #76)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Return") PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13) // 4D (key #77)
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("\\  |") PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|') // 4E (key #78)
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT)) // 4F (key #79)

	PORT_START("ROW5")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Home") PORT_CODE(KEYCODE_HOME) PORT_CHAR(UCHAR_MAMEKEY(HOME)) // 50 (key #80)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Space") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ') // 51 (key #81)
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Z") PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z') // 52 (key #82)
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("X") PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X') // 53 (key #83)
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C') // 54 (key #84)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("V") PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V') // 55 (key #85)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B') // 56 (key #86)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("N") PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N') // 57 (key #87)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M') // 58 (key #88)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(",  <") PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<') // 59 (key #89)
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Print") PORT_CODE(KEYCODE_PRTSCR) PORT_CHAR(UCHAR_MAMEKEY(PRTSCR)) // 5A (key #90)
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(".  >") PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>') // 5B (key #91)
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("/  ?") PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?') // 5C (key #92)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_UNUSED) // 5D
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_UNUSED) // 5E
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_UNUSED) // 5F

	PORT_START("ROW6")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN)) // 60 (key #96)
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_UNUSED) // 61
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_UNUSED) // 62
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_UNUSED) // 63
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Brk  Pause") PORT_CODE(KEYCODE_PAUSE) PORT_CHAR(UCHAR_MAMEKEY(PAUSE)) // 64 (key #100)
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1) PORT_CHAR(UCHAR_MAMEKEY(F1)) // 65 (key #101)
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2) PORT_CHAR(UCHAR_MAMEKEY(F2)) // 66 (key #102)
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3)) // 67 (key #103)
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4) PORT_CHAR(UCHAR_MAMEKEY(F4)) // 68 (key #104)
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_UNUSED) // 69
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_UNUSED) // 6A
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_UNUSED) // 6B
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_UNUSED) // 6C
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_UNUSED) // 6D
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_UNUSED) // 6E
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_UNUSED) // 6F

	PORT_START("MODIFIERS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Ctrl") PORT_CODE(KEYCODE_LCONTROL) PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Alt") PORT_CODE(KEYCODE_LALT) PORT_CODE(KEYCODE_RALT)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Caps Lock") PORT_CODE(KEYCODE_CAPSLOCK) PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
INPUT_PORTS_END

} // anonymous namespace


tipc_keyboard_device::tipc_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, TIPC_KEYBOARD, tag, owner, clock)
	, device_buffered_serial_interface(mconfig, *this)
	, device_matrix_keyboard_interface(mconfig, *this, "ROW0", "ROW1", "ROW2", "ROW3", "ROW4", "ROW5", "ROW6")
	, m_txd_cb(*this)
	, m_cts_cb(*this)
	, m_modifiers(*this, "MODIFIERS")
	, m_response_timer(nullptr)
	, m_command(0)
	, m_last_mode(0x78)
	, m_locked(false)
	, m_repeat(true)
	, m_caps_lock(false)
	, m_caps_key(false)
{
}

ioport_constructor tipc_keyboard_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(tipc_keyboard);
}

void tipc_keyboard_device::device_start()
{
	m_response_timer = timer_alloc(FUNC(tipc_keyboard_device::respond), this);

	save_item(NAME(m_command));
	save_item(NAME(m_last_mode));
	save_item(NAME(m_locked));
	save_item(NAME(m_repeat));
	save_item(NAME(m_caps_lock));
	save_item(NAME(m_caps_key));
}

void tipc_keyboard_device::device_reset()
{
	clear_fifo();
	receive_register_reset();
	transmit_register_reset();

	set_data_frame(1, 8, PARITY_EVEN, STOP_BITS_1);
	set_tra_rate(2441); // 2440 baud +-1.5% (p. 2-3)
	set_rcv_rate(305);  // 305 baud +-1.5% (p. 2-4)

	m_txd_cb(1);
	m_cts_cb(0);

	m_response_timer->adjust(attotime::never);
	reset_state();
	m_caps_key = false;

	reset_key_state();
	start_processing(attotime::from_hz(2'400));
}

void tipc_keyboard_device::reset_state()
{
	// power-up defaults: repeat-action on, unlocked, no modes (Table 2-1)
	m_locked = false;
	m_repeat = true;
	m_caps_lock = false;
	m_last_mode = 0x78;
	typematic_stop();
}

void tipc_keyboard_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void tipc_keyboard_device::received_byte(u8 byte)
{
	m_command = byte;
	m_response_timer->adjust(attotime::from_msec(5));
}

TIMER_CALLBACK_MEMBER(tipc_keyboard_device::respond)
{
	switch (m_command)
	{
	case 0x00: // power-up self-test, install defaults
	case 0x07: // reset (same as 00)
		clear_fifo();
		reset_state();
		transmit_byte(0x70);
		break;

	case 0x01: // repeat-action on
		m_repeat = true;
		transmit_byte(0x70);
		break;

	case 0x02: // repeat-action off
		m_repeat = false;
		typematic_stop();
		transmit_byte(0x70);
		break;

	case 0x03: // lock keyboard
		m_locked = true;
		typematic_stop();
		transmit_byte(0x70);
		break;

	case 0x04: // unlock keyboard
		m_locked = false;
		transmit_byte(0x70);
		break;

	case 0x05: // keyclick on (not supported by stock hardware)
	case 0x06: // keyclick off
		transmit_byte(0x70);
		break;

	case 0x08: // return version (2-byte code; values OCR-ambiguous)
		transmit_byte(0x70);
		transmit_byte(0x73);
		break;

	default: // unknown commands are ignored, the system unit retries
		logerror("unknown command %02X\n", m_command);
		break;
	}
}

u8 tipc_keyboard_device::mode_byte() const
{
	return 0x78 | (m_modifiers->read() & 0x07) | (m_caps_lock ? 0x80 : 0x00);
}

void tipc_keyboard_device::send_key(u8 code)
{
	u8 const mode = mode_byte();
	if (mode != m_last_mode)
	{
		transmit_byte(mode);
		m_last_mode = mode;
	}
	transmit_byte(code);
}

void tipc_keyboard_device::key_make(u8 row, u8 column)
{
	u8 const code = row * 16 + column;
	if (m_locked || !code || (code > 0x68))
		return;

	send_key(code);

	// only the most recently pressed key repeats
	if (m_repeat)
		typematic_start(row, column, attotime::from_msec(500), attotime::from_hz(15));
	else
		typematic_stop();
}

void tipc_keyboard_device::key_repeat(u8 row, u8 column)
{
	if (!m_locked)
		transmit_byte(0x80 | (row * 16 + column));
}

void tipc_keyboard_device::scan_complete()
{
	// CAPS LOCK is a toggle kept by the keyboard and reported in the mode byte
	bool const caps = BIT(m_modifiers->read(), 3);
	if (caps && !m_caps_key)
		m_caps_lock = !m_caps_lock;
	m_caps_key = caps;
}
