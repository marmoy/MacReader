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
#include <ctime>
#include <iostream>

#include "options.h"
#include "Encoder_impl.h"
using namespace std;

namespace gr {
namespace RFID_MAC {

Encoder::Encoder::sptr
Encoder::make()
{
	return gnuradio::get_initial_sptr(new Encoder_impl());
}

/*
 * The private constructor
 */
Encoder_impl::Encoder_impl()
: gr::block("Encoder",
		gr::io_signature::make(1, 1, sizeof(int)),
		gr::io_signature::make(1, 1, sizeof(float)))
{
	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

/*
 * Our virtual destructor.
 */
Encoder_impl::~Encoder_impl()
{}

int Encoder_impl::general_work (int noutput_items,
		gr_vector_int &ninput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items){
	const int *in = (const int *) input_items[0];
	float *out = (float *) output_items[0];
	int written = 0;

	if (get_tag_value("frame", ninput_items[0]) == FRAMES::PREAMBLE) {
		add_cw(out, written, true);
		add_preamble(out, written);
	}
	else {
		add_frame_sync(out, written);
	}
	add_command(out, written, in, ninput_items[0]);

	add_cw(out, written, false);

	consume_each(ninput_items[0]);
	return written;
}

// Encodes the input buffer and adds the encoded result to the output buffer
void Encoder_impl::add_command(float* out, int &written, const int* in, int size_in){
	for(int command_element=0; command_element<size_in; command_element++){
		if (in[command_element] == 0) {
			add_data_0(out, written);
		}
		else {
			add_data_1(out, written);
		}
	}
}

/* ==============================================================
 * STREAM TAG METHODS
 * ==============================================================*/
// Returns the tag value corresponding to the provided tag key.
int Encoder_impl::get_tag_value(std::string tag_key, int ninput_items){
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items);
	if (tags.empty()) {
		cout << "Failure at " << this->name() << " due to missing stream tag: " << tag_key << endl;
		return 0;
	}
	else {
		return pmt::to_uint64(tags[0].value);
	}
}

/* ==============================================================
 * RF SYMBOL METHODS
 * ==============================================================*/
void Encoder_impl::add_preamble(float* out, int &written){
	add_frame_sync(out, written);
	add_trcal(out, written);
}

void Encoder_impl::add_frame_sync(float* out, int &written){
	add_delim(out, written);
	add_data_0(out, written);
	add_rtcal(out, written);
}

void Encoder_impl::add_trcal(float* out, int &written){
	fill(out, written, 1, Options::get_TRcal_us()-Options::get_PW_us());
	add_pw(out, written);
}

void Encoder_impl::add_rtcal(float* out, int &written){
	fill(out, written, 1,Options::get_RTcal_us()-Options::get_PW_us());
	add_pw(out, written);
}

void Encoder_impl::add_data_1(float* out, int &written){
	fill(out, written, 1, Options::get_DATA1_us()-Options::get_PW_us());
	add_pw(out, written);
}

void Encoder_impl::add_data_0(float* out, int &written){
	fill(out, written, 1, Options::get_DATA0_us()-Options::get_PW_us());
	add_pw(out, written);
}

void Encoder_impl::add_pw(float* out, int &written){
	fill(out, written, 0, Options::get_PW_us());
}

void Encoder_impl::add_delim(float* out, int &written){
	fill(out, written, 0, Options::get_DELIM_us());
}

void Encoder_impl::add_cw(float* out, int &written, bool explicit_transmit){				//Continuous wave (CW) = High(CW time)
	double cw_len_us = 1; 	//Due to physical characteristics of the USRP, we only need to transmit a single 1 to get a tail CW. This allows the scheduler to move to the receive chain much earlier
	if (explicit_transmit) {
		cw_len_us = Options::CW_HEAD_us;
	}
	fill(out, written, 1, cw_len_us);
}

void Encoder_impl::fill(float* out, int &written, int value, int size){
	for (int i = 0; i < size; i++) {
		out[written+i] = value;
	}
	written += size;
}
} /* namespace RFID_MAC */
} /* namespace gr */

