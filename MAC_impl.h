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

#ifndef INCLUDED_RFID_MAC_MAC_IMPL_H
#define INCLUDED_RFID_MAC_MAC_IMPL_H

#include "include/MAC.h"
#include "options.h"

#include <iterator>
#include <string>
#include <iostream>
#include <algorithm>

using namespace std;

namespace gr {
  namespace RFID_MAC {

class MAC_impl : public MAC
{
private:
	static const int US_PER_SECOND = 1000000;

	string d_divide_ratio;
	string d_miller_encoding;
	string d_trext;
	string d_sel;
	string d_session;
	string d_target;
	string d_q;
	double divide_ratio_numeric = 0;
	double miller_encoding_numeric = 0;
	string query_field = "1000";
	string qrep_field = "00";
	string ack_field = "01";
	string nak_field = "11000000";

	pmt::pmt_t message;

	//Configuration
	int INIT_QFP = 2;
	pmt::pmt_t control_port_in = pmt::mp("control_port_in");
	pmt::pmt_t log_port = pmt::mp("log_port");
	pmt::pmt_t reset_gate_message_port = pmt::mp("reset_gate_message_port");
	pmt::pmt_t update_blf_message_port = pmt::mp("update_blf_message_port");
	pmt::pmt_t update_t1_message_port = pmt::mp("update_t1_message_port");
	pmt::pmt_t update_miller_message_port = pmt::mp("update_miller_message_port");

	//State
	int previous_command_sent = COMMANDS::IDLE;
	int number_of_slots = 0;
	int current_slot = 0;
	int cycles = 0;
	int d_num_tags_read = 0;
	bool message_received = true;
	bool config_changed = true;

	/*==========================================================================
	 * Link timing parameters - see table 6.13 and figure 6.16 in the standard
	 * =========================================================================*/
	double get_T_pri_us(){ return seconds_to_microseconds(1.0/get_BLF());}
	double get_T1_nominal_us(){ return std::max(10.0*get_T_pri_us(), Options::get_RTcal_us() );} // See table 6.13 in the standard
	double get_T2_max_us(){ return 20.0*get_T_pri_us();}
	double get_T2_min_us(){ return 3.0*get_T_pri_us();}
	double get_T4_us(){ return 10.0*2.0*Options::get_RTcal_us();}
	double get_BLF(){ return divide_ratio_numeric/microseconds_to_seconds(Options::TRcal_us);} // BLF = Backscatter Link Frequency. (WISP is hard-coded to 256kHz).
	double get_DATA_RATE(){ return get_BLF()/miller_encoding_numeric;} //Tag-to-Interrogator. See table 6.10.

	//command methods
	void add_query(int* out, int &written);
	void add_qrep(int* out, int &written);
	void add_ack(int* out, int &written, string rn16);
	void add_nak(int* out, int &written);
	void transition(int next_state);
	void inventory_round_SM(int*out, int &written);
	void query_loop_SM(int*out, int &written);
	void add_to_out(int* out, int &written, string in);
	void get_crc(int* out, int &written, string cmd, string &crc);
	void response_handler(pmt::pmt_t response_msg);
	void error_handler(pmt::pmt_t response_msg);
	void reconfigure();
	int get_command_us(string cmd, int frame_type);

	//CRC methods
	vector<int> calculate_crc(int * packet);
	bool isValidCRC(std::string bits, int num_bits);

	double seconds_to_microseconds(double seconds){	return seconds*US_PER_SECOND; }
	double microseconds_to_seconds(double microseconds){ return microseconds/US_PER_SECOND;	}
	double microseconds_to_samples(double microseconds, double sample_rate){ return sample_rate*microseconds_to_seconds(microseconds); }
public:
	MAC_impl();
	~MAC_impl();
	int general_work (int noutput_items,
			gr_vector_int &ninput_items, //array containing the number of input items for each port of the block. Index 0 corresponds to port 0, 1 to 1 and so on.
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};


  } // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_MAC_IMPL_H */

