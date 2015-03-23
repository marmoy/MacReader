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

#ifndef INCLUDED_RFID_MAC_SYNCHRONIZER_IMPL_H
#define INCLUDED_RFID_MAC_SYNCHRONIZER_IMPL_H

#include "include/Synchronizer.h"
#include "options.h"
#include <iostream>
using namespace std;

namespace gr {
namespace RFID_MAC {

class Synchronizer_impl : public Synchronizer
{
private:
	int d_sampling_rate;
	bool do_synchronize = false;
	int previous_zc = 0;
	int current_zc = 0;
	int previous_zc_absolute = 0;
	double d_pivot = 0;
	const pmt::pmt_t update_blf_message_port = pmt::mp("update_blf_message_port");

	void synchronize (pmt::pmt_t pdu);
	void propagate_tags(int ninput_items, int written);
	bool tag_exists_in_range(std::string key, int ninput_items);
	void update_blf(pmt::pmt_t msg);
public:
	Synchronizer_impl(int sampling_rate);
	~Synchronizer_impl();
	int general_work (int noutput_items,
			gr_vector_int &ninput_items,
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};
} // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_SYNCHRONIZER_IMPL_H */

