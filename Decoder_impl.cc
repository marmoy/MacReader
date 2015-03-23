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
#include "Decoder_impl.h"

namespace gr {
namespace RFID_MAC {

Decoder::sptr Decoder::make() {
	return gnuradio::get_initial_sptr(new Decoder_impl());
}

// The private constructor
Decoder_impl::Decoder_impl() :
				gr::block("Decoder", gr::io_signature::make(1, 1, sizeof(int)),
						gr::io_signature::make(0, 0, 0)) {
	tag_dict = pmt::make_dict();
	message_port_register_out(decoder_message_out);
	message_port_register_in(update_miller_message_port);
	set_msg_handler(update_miller_message_port,
			boost::bind(&Decoder_impl::set_encoding, this, _1));
	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
}

// Our virtual destructor.
Decoder_impl::~Decoder_impl() {}

int Decoder_impl::general_work(int noutput_items,
		gr_vector_int &ninput_items, //array containing the number of input items for each port of the block. Index 0 corresponds to port 0, 1 to 1 and so on.
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {
	const int *in = (const int *) input_items[0];
	propagate_tags(ninput_items[0]);

	if (!do_decode) { // Wait for beginning of encoded response
		do_decode = tag_exists_in_range("response_beginning", ninput_items[0]);
	}
	//TODO: reverse decoding, so we look for dummy bit and go back
	if (do_decode) {
		for (int i = 0; i < ninput_items[0]; i++) { // Buffer entire encoded response before decoding
			buffer[buffer_len++] = in[i];
		}

		if (tag_exists_in_range("response_ending", ninput_items[0])) { // We have buffered the whole response, time to decode
			int payload_offset = 0;
			/*cout << endl;
			for (int i = 0; i< buffer_len; i++) {
				if (buffer[i]==1) {
					cout << 1;
				}
				else {
					cout << 0;
				}
			}
			cout << endl;*/
			for (size_t i = 0; i < buffer_len - preamble.size(); i++) {
				double sum = 0;
				double total_pwr = 0;

				for (size_t j = 0; j < preamble.size(); j++) {//Calculate correlation with data-1 symbol
					total_pwr += fabs(buffer[i + j]);
					sum += preamble[j] * (buffer[i + j]);
				}

				float score = fabs(sum) / total_pwr;

				if (score == 1) {//If the response correlates with a data-1 symbol
					preamble_found = true;
					payload_offset = i + preamble.size();
					break;
				}
			}

			if (preamble_found) { // We now know the index of the first miller symbol and can start decoding.
				std::string out = "";

				for (int i = payload_offset;
						i < buffer_len/* - data_1.size()*/; i += data_1.size()) {
					double sum = 0;
					double total_pwr = 0;

					for (size_t j = 0; j < data_1.size(); j++) {//Calculate correlation with data-1 symbol
						total_pwr += fabs(buffer[i + j]);
						sum += data_1[j] * (buffer[i + j]);
					}

					float cross_correlation = fabs(sum) / total_pwr;
					if (cross_correlation == 1) {
						out.append(1, '1');
					} else {
						out.append(1, '0');
					}
				}
				cout << "Response: " << out << endl;
				if (isValidResponse(out)) {
					out.pop_back(); // Remove dummy bit from response
					inject_tag("response", out);
				} else {
					inject_tag("failure", ERRORS::INVALID_RESPONSE);
				}
			} else {
				inject_tag("failure", ERRORS::NO_PREAMBLE);
			}
			publish();
		}
	}
	consume_each(ninput_items[0]);
	return 0;
}

bool Decoder_impl::isValidResponse(std::string response) {
	return response[16]=='1';
}

void Decoder_impl::publish() {
	message_port_pub(decoder_message_out, tag_dict);
	reset_block();
}

void Decoder_impl::propagate_tags(int ninput_items) {
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items);

	BOOST_FOREACH(const gr::tag_t &tag, tags){
		tag_dict = pmt::dict_add(tag_dict, tag.key, tag.value);
	}
}

void Decoder_impl::reset_block() {
	tag_dict = pmt::make_dict();
	buffer_len = 0;
	preamble_found = false;
	do_decode = false;
}

bool Decoder_impl::tag_exists_in_range(std::string key,
		int ninput_items) {
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items,
			pmt::string_to_symbol(key));
	if (!tags.empty()) {
		cout << "Decoder received tag: " << key << endl;
	}
	return !tags.empty();
}

void Decoder_impl::set_encoding(pmt::pmt_t encoding) {

	if (pmt::to_uint64(encoding) == 8) {
		/*	preamble = m8_preamble_vec;
		 data_1 = m8_data_one_vec;*/
	} else if (pmt::to_uint64(encoding) == 4) {
		cout << "encoding set at decoder" << endl;
		preamble = m4_preamble_vec;
		data_1 = m4_data_one_vec;
	} else if (pmt::to_uint64(encoding) == 2) {
		/*preamble = m2_preamble_vec;
		 data_1 = m2_data_one_vec;*/
	}
}
} /* namespace RFID_MAC */
} /* namespace gr */
