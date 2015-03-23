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

#ifndef INCLUDED_RFID_MAC_GATEKEEPER_IMPL_H
#define INCLUDED_RFID_MAC_GATEKEEPER_IMPL_H

#include "include/Gatekeeper.h"
#include "options.h"
#include <iostream>
#include <volk/volk.h>
#include <gnuradio/blocks/file_sink.h>
#include <ctime>


using namespace std;

namespace gr {
namespace RFID_MAC {

class Gatekeeper_impl : public Gatekeeper
{
private:
	static const int US_PER_SECOND = 1000000;

	int T1_MAX_LENGTH;
	int command_start_index = 0;
	int t1_start_index = 0;
	int response_start_index = 0;
	int response_end_index = 0;
	int response_count = 0;
	int slice_length = 0;
	int samples_without_change = 0;
	int skip_to_index = 0;
	pmt::pmt_t tag_dict;

	float std_dev_signal = 0;
	float std_dev_noise = 0;

	int USRP_SIGNAL_SETTLING_LENGTH; //The delay in samples between the USRP starts to change until change takes place
	const double COMMAND_EDGE_SLOPE = 0.5; //The rate of change that characterizes a subcarrier cycle
	const double TAG_RESPONSE_EDGE_SLOPE = 0.01; //The rate of change that characterizes a subcarrier cycle

	int command_length = 0;

	/*min_nitems_required determines the minimum number of input items received from the previous block
	 * if this number gets too large, there's a high probability that we overshoot the end of the response.
	 * That would lead to the processing being delayed by the equivalent time as the overshoot*/
	int min_nitems_required = 64;

	const pmt::pmt_t reset_gate_message_port = pmt::mp("reset_gate_message_port");
	const pmt::pmt_t gatekeeper_failure_port_out = pmt::mp("gatekeeper_failure_port_out");
	const pmt::pmt_t update_blf_message_port = pmt::mp("update_blf_message_port");
	const pmt::pmt_t update_t1_message_port = pmt::mp("update_t1_message_port");


	enum States{FIND_CW_RISING_EDGE, FIND_COMMAND_RISING_EDGE, SKIP_COMMAND, FIND_RESPONSE_RISING_EDGE, FIND_RESPONSE_FALLING_EDGE, WAIT};
	bool resume = false;
	double d_sample_rate;
	int state = FIND_CW_RISING_EDGE;

	double get_SNR(const float *signal, const float *noise, double nitems);
	void write_to_file(int nitems, const float* inbuf);
	void reset_gate(pmt::pmt_t command_us);
	void update_blf(pmt::pmt_t blf);
	void update_t1(pmt::pmt_t t1_nominal_us);
	void handle_failure(int error);
	void advance_state(int next_state);

	template<class TYPE>
	void inject_tag(string key, TYPE value, int offset)
	{
		cout << "Gate: " << key << " = " << value << endl;
		tag_dict = pmt::dict_add(tag_dict, pmt::string_to_symbol(key), pmt::make_any(value));
		this->add_item_tag(0, this->nitems_written(0), pmt::string_to_symbol(key), pmt::make_any(value), pmt::string_to_symbol(this->name()));
	}

	double microseconds_to_seconds(double microseconds){ return microseconds/US_PER_SECOND;	}
	double microseconds_to_samples(double microseconds, double sample_rate){ return sample_rate*microseconds_to_seconds(microseconds); }

public:
	Gatekeeper_impl(int sampling_rate);
	~Gatekeeper_impl();
	void forecast(int noutput_items, gr_vector_int &ninput_items_required);
	int general_work(int noutput_items,
			gr_vector_int &ninput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};

} // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_GATEKEEPER_IMPL_H */

