/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <gnuradio/io_signature.h>
#include "MAC_impl.h"

namespace gr {
namespace RFID_MAC {

MAC::sptr
MAC::make()
{
	return gnuradio::get_initial_sptr(new MAC_impl());
}

/*
 * The private constructor
 */
MAC_impl::MAC_impl()
: gr::block("MAC",
		gr::io_signature::make(0, 0, 0),
		gr::io_signature::make(1, 1, sizeof(int)))
{
	message = pmt::make_dict();

	// initialize configuration
	if (Options::divide_ratio_default == DIVIDE_RATIO::DR0) {
		divide_ratio_numeric = 8;
	}
	else if (Options::divide_ratio_default == DIVIDE_RATIO::DR1){
		divide_ratio_numeric = 64/3;
	}
	else {
		cout << "Invalid divide ratio" << endl;
	}

	if (Options::miller_encoding_default == MILLER_ENCODING::M1) {
		miller_encoding_numeric = 1;
	}
	else if (Options::miller_encoding_default == MILLER_ENCODING::M2) {
		miller_encoding_numeric = 2;
	}
	else if (Options::miller_encoding_default == MILLER_ENCODING::M4) {
		miller_encoding_numeric = 4;
	}
	else if (Options::miller_encoding_default == MILLER_ENCODING::M8) {
		miller_encoding_numeric = 8;
	}
	else {
		cout << "Invalid miller encoding" << endl;
	}

	d_divide_ratio = bitset<1>(Options::divide_ratio_default).to_string('0','1');
	d_miller_encoding = bitset<2>(Options::miller_encoding_default).to_string('0','1');
	d_trext = bitset<1>(Options::trext_default).to_string('0','1');
	d_sel = bitset<2>(Options::sel_default).to_string('0','1');
	d_session = bitset<2>(Options::session_default).to_string('0','1');
	d_target = bitset<1>(Options::target_default).to_string('0','1');
	d_q = bitset<4>(Options::q_default).to_string('0','1');

	//register message queues
	message_port_register_out(reset_gate_message_port);
	message_port_register_out(update_blf_message_port);
	message_port_register_out(update_t1_message_port);
	message_port_register_out(update_miller_message_port);

	message_port_register_in(control_port_in);
	set_msg_handler(control_port_in, boost::bind(&MAC_impl::response_handler, this, _1));

	//We handle our own tag propagation (probably redundant, but it is a pain to debug tags, so just to be safe)
	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

/*
 * Our virtual destructor.
 */
MAC_impl::~MAC_impl()
{
}

void MAC_impl::response_handler(pmt::pmt_t msg){
	message = msg;
	message_received = true;
}

int
MAC_impl::general_work (int noutput_items,
		gr_vector_int &ninput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items)
{
	if (!message_received) {
		return 0;
	}

	int *out = (int *) output_items[0];
	int written = 0;

	reconfigure(); //handles channel metadata

	inventory_round_SM(out, written);
	//query_loop_SM(out, written);
	return written;
}

void MAC_impl::query_loop_SM(int*out, int &written){
	add_query(out, written);
}

void MAC_impl::inventory_round_SM(int*out, int &written){
	bool rx_failed = pmt::dict_has_key(message, pmt::string_to_symbol("failure"));

	switch (previous_command_sent) {
	case COMMANDS::IDLE:
	{
		add_query(out, written);
		break;
	}
	case COMMANDS::QREP: //fall through to QUERY state
	case COMMANDS::QUERY:
	{
		if (!rx_failed) {
			string response = boost::any_cast<std::string>(pmt::any_ref(pmt::dict_ref(message, pmt::string_to_symbol("response"), pmt::PMT_NIL)));
			add_ack(out, written, response);
		}
		else {
			add_qrep(out, written);
		}
		break;
	}
	case COMMANDS::ACK:
	{
		if (rx_failed) {
			add_qrep(out, written);
		}
		else {
			string response = boost::any_cast<std::string>(pmt::any_ref(pmt::dict_ref(message, pmt::string_to_symbol("response"), pmt::PMT_NIL)));
			if (!isValidCRC(response, response.size())) {
				add_nak(out, written);
			}
			else {
				add_qrep(out, written);
			}
		}
		break;
	}
	case COMMANDS::NAK: // RX is expected to timeout on nak, since no response is expected. Just transmit qrep
	{
		add_qrep(out, written);
		break;
	}
	case FINAL: // We have reached the last cycle
	{
		break;
	}
	default:
		break;
	}
}

/* ==============================================================
 * COMMAND METHODS
 * ==============================================================*/
void MAC_impl::add_query(int*out, int &written){
	string query = query_field + d_divide_ratio + d_miller_encoding + d_trext + d_sel + d_session + d_target + d_q;
	string crc;
	get_crc(out, written, query, crc);
	query.append(crc.begin(), crc.end());
	add_to_out(out, written, query);

	this->add_item_tag(0, this->nitems_written(0), pmt::string_to_symbol("frame"), pmt::from_uint64(FRAMES::PREAMBLE));
	message_port_pub(reset_gate_message_port, pmt::from_uint64(get_command_us(query, FRAMES::PREAMBLE)));

	transition(COMMANDS::QUERY);
}

void MAC_impl::add_qrep(int*out, int &written){
	string qrep = qrep_field + d_session;
	add_to_out(out, written, qrep);

	this->add_item_tag(0, this->nitems_written(0), pmt::string_to_symbol("frame"), pmt::from_uint64(FRAMES::FRAMESYNC));
	message_port_pub(reset_gate_message_port, pmt::from_uint64(get_command_us(qrep, FRAMES::FRAMESYNC)));

	transition(COMMANDS::QREP);
}

void MAC_impl::add_ack(int*out, int &written, std::string rn16){
	string ack = ack_field + rn16;
	add_to_out(out, written, ack);

	this->add_item_tag(0, this->nitems_written(0), pmt::string_to_symbol("frame"), pmt::from_uint64(FRAMES::FRAMESYNC));
	message_port_pub(reset_gate_message_port, pmt::from_uint64(get_command_us(ack, FRAMES::FRAMESYNC)));

	transition(COMMANDS::ACK);
}

void MAC_impl::add_nak(int*out, int &written){
	string nak = nak_field;
	add_to_out(out, written, nak);

	this->add_item_tag(0, this->nitems_written(0), pmt::string_to_symbol("frame"), pmt::from_uint64(FRAMES::FRAMESYNC));
	message_port_pub(reset_gate_message_port, pmt::from_uint64(get_command_us(nak, FRAMES::FRAMESYNC)));

	transition(COMMANDS::NAK);
}

void MAC_impl::add_to_out(int* out, int &written, string in){
	for (size_t i = 0; i < in.size(); i++) {
		out[written+i]=in[i]-'0';
	}
	written +=in.size();
}

void MAC_impl::transition(int next_state){
	message_received = false;
	cycles++;
	if (Options::MAX_CYCLES > 0 && cycles > Options::MAX_CYCLES) {
		previous_command_sent = FINAL;
	}
	else {
		previous_command_sent = next_state;
	}
	cout << "Reader state: " << previous_command_sent << endl;
}

//Note: only publish the absolute minimum necessary. If a parameter has not changed value, do not publish it, it has large overhead.
void MAC_impl::reconfigure(){
	if (config_changed) {
		message_port_pub(update_blf_message_port, pmt::from_double(get_BLF()));
		message_port_pub(update_t1_message_port, pmt::from_double(get_T1_nominal_us()));
		message_port_pub(update_miller_message_port, pmt::from_uint64(miller_encoding_numeric));
		config_changed = false;
	}
}

int MAC_impl::get_command_us(string cmd, int frame_type)
{
	int frame_us = 0;
	if (frame_type == FRAMES::FRAMESYNC) {
		frame_us = Options::get_FRAME_SYNC_us();
	}
	else if (frame_type == FRAMES::PREAMBLE) {
		frame_us = Options::get_PREAMBLE_us();
	}
	else {
		cout << "Invalid frame type" << endl;
	}
	size_t ones = std::count(cmd.begin(), cmd.end(), '1');
	return frame_us + (ones)*Options::get_DATA1_us()+(cmd.size()-ones)*Options::get_DATA0_us();
}

/* ==============================================================
 * CRC METHODS
 * ==============================================================*/
void MAC_impl::get_crc(int* out, int &written, string cmd, string &crc)
{
	crc = "10010";
	for(size_t i = 0; i < cmd.size(); i++){
		string tmp = "00000";
		tmp[4] = crc[3];
		if(crc[4] == '1'){
			if (cmd[i] == '1'){
				tmp[0] = '0';
				tmp[1] = crc[0];
				tmp[2] = crc[1];
				tmp[3] = crc[2];
			}
			else{
				tmp[0] = '1';
				tmp[1] = crc[0];
				tmp[2] = crc[1];
				if(crc[2] == '1'){
					tmp[3] = '0';
				}
				else{
					tmp[3] = '1';
				}
			}
		}
		else{
			if (cmd[i] == '1'){
				tmp[0] = '1';
				tmp[1] = crc[0];
				tmp[2] = crc[1];
				if(crc[2] == '1'){
					tmp[3] = '0';
				}
				else{
					tmp[3] = '1';
				}
			}
			else{
				tmp[0] = '0';
				tmp[1] = crc[0];
				tmp[2] = crc[1];
				tmp[3] = crc[2];
			}
		}
		crc = tmp;
	}
	std::reverse(crc.begin(), crc.end());
}

bool MAC_impl::isValidCRC(std::string bits, int num_bits){
	register unsigned short i, j;
	register unsigned short crc_16, rcvd_crc;
	unsigned char * data;
	int num_bytes = num_bits / 8;
	data = (unsigned char* )malloc(num_bytes );
	int mask;

	for(i = 0; i < num_bytes; i++){
		mask = 0x80;
		data[i] = 0;
		for(j = 0; j < 8; j++){
			if (bits[(i * 8) + j] == '1'){
				data[i] = data[i] | mask;
			}
			mask = mask >> 1;
		}
	}
	rcvd_crc = (data[num_bytes - 2] << 8) + data[num_bytes -1];

	crc_16 = 0xFFFF;
	for (i=0; i < num_bytes - 2; i++) {
		crc_16^=data[i] << 8;
		for (j=0;j<8;j++) {
			if (crc_16&0x8000) {
				crc_16 <<= 1;
				crc_16 ^= 0x1021; // (CCITT) x16 + x12 + x5 + 1
			}
			else {
				crc_16 <<= 1;
			}
		}
	}
	crc_16 = ~crc_16;

	return rcvd_crc == crc_16;
}
} /* namespace RFID_MAC */
} /* namespace gr */
