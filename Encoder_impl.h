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

#ifndef INCLUDED_RFID_MAC_ENCODER_IMPL_H
#define INCLUDED_RFID_MAC_ENCODER_IMPL_H

#include "include/Encoder.h"

namespace gr {
namespace RFID_MAC {

class Encoder_impl : public Encoder
{
private:
	void add_command(float* out, int &written, const int* in, int size_in);
	void add_preamble(float* out, int &written);
	void add_frame_sync(float* out, int &written);
	void add_delim(float* out, int &written);
	void add_trcal(float* out, int &written);
	void add_rtcal(float* out, int &written);
	void add_data_1(float* out, int &written);
	void add_data_0(float* out, int &written);
	void add_pw(float* out, int &written);
	void add_cw(float* out, int &written, bool explicit_transmit);
	void fill(float* out, int &written, int value, int size);

	int get_tag_value(std::string tag_key, int ninput_items);

public:
	Encoder_impl();
	~Encoder_impl();
	int general_work (int noutput_items,
			gr_vector_int &ninput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};

} // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_ENCODER_IMPL_H */

