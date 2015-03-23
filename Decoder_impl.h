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

#ifndef INCLUDED_RFID_MAC_DECODER_IMPL_H
#define INCLUDED_RFID_MAC_DECODER_IMPL_H

#include "include/Decoder.h"
#include <gnuradio/blocks/pdu.h>
#include <vector>
#include "options.h"
#include <iostream>
using namespace std;

namespace gr {
namespace RFID_MAC {

class Decoder_impl : public Decoder
{
private:
	pmt::pmt_t decoder_message_out = pmt::mp("decoder_message_out");
	pmt::pmt_t update_miller_message_port = pmt::mp("update_miller_message_port");
	pmt::pmt_t tag_dict;

	bool do_decode = false;
	int buffer_len = 0;
	int number_of_bits_to_decode = 0;
	int buffer[2000];
	bool preamble_found = false;

	vector<int> data_1;
	vector<int> preamble;
	vector<int> m4_data_one_vec = {1,-1,1,-1,-1,1,-1,1};
	vector<int> m4_preamble_vec = {-1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,1,-1};

	void publish();
	void propagate_tags(int ninput_items);
	bool isValidResponse(std::string response);
	void set_encoding(pmt::pmt_t tag_dict);
	bool tag_exists_in_range(std::string key, int ninput_items);
	void reset_block();
	template<class TYPE>
	void inject_tag(string key, TYPE value)
	{
		cout << "Decoder: " << key << " = " << value << endl;
		tag_dict = pmt::dict_add(tag_dict, pmt::string_to_symbol(key), pmt::make_any(value));
	}
public:
	Decoder_impl();
	~Decoder_impl();
	int general_work (int noutput_items,
			gr_vector_int &ninput_items, //array containing the number of input items for each port of the block. Index 0 corresponds to port 0, 1 to 1 and so on.
			gr_vector_const_void_star &input_items,
			gr_vector_void_star &output_items);
};
} // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_DECODER_IMPL_H */

