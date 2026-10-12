// license:BSD-3-Clause
// copyright-holders:TODO (author name)
/***************************************************************************

    Texas Instruments Professional Computer keyboard (HLE)

***************************************************************************/

#ifndef MAME_TI_TIPC_KBD_H
#define MAME_TI_TIPC_KBD_H

#pragma once

#include "machine/keyboard.h"
#include "diserial.h"


class tipc_keyboard_device : public device_t,
		public device_buffered_serial_interface<16U>,
		protected device_matrix_keyboard_interface<7U>
{
public:
	tipc_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto txd_handler() { return m_txd_cb.bind(); }
	auto cts_handler() { return m_cts_cb.bind(); }

	void rxd_w(int state) { device_buffered_serial_interface::rx_w(state); }

protected:
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_serial_interface
	virtual void tra_callback() override;
	virtual void received_byte(u8 byte) override;

	// device_matrix_keyboard_interface
	virtual void key_make(u8 row, u8 column) override;
	virtual void key_repeat(u8 row, u8 column) override;
	virtual void scan_complete() override;

private:
	TIMER_CALLBACK_MEMBER(respond);

	void reset_state();
	u8 mode_byte() const;
	void send_key(u8 code);

	devcb_write_line m_txd_cb;
	devcb_write_line m_cts_cb;
	required_ioport m_modifiers;
	emu_timer *m_response_timer;

	u8 m_command;
	u8 m_last_mode;
	bool m_locked;
	bool m_repeat;
	bool m_caps_lock;
	bool m_caps_key;
};

DECLARE_DEVICE_TYPE(TIPC_KEYBOARD, tipc_keyboard_device)

#endif // MAME_TI_TIPC_KBD_H
