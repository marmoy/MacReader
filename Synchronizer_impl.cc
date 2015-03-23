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
#include "Synchronizer_impl.h"

namespace gr {
namespace RFID_MAC {

Synchronizer::sptr
Synchronizer::make(int sampling_rate)
{
	return gnuradio::get_initial_sptr
			(new Synchronizer_impl(sampling_rate));
}

/*
 * The private constructor
 */
Synchronizer_impl::Synchronizer_impl(int sampling_rate)
: gr::block("Synchronizer",
		gr::io_signature::make(1, 1, sizeof(int)),
		gr::io_signature::make(1, 1, sizeof(int)))
{
	d_sampling_rate = sampling_rate;
	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
	message_port_register_in(update_blf_message_port);
	set_msg_handler(update_blf_message_port, boost::bind(&Synchronizer_impl::update_blf, this, _1));
}
/*
 * Our virtual destructor.
 */
Synchronizer_impl::~Synchronizer_impl()
{
}

int
Synchronizer_impl::general_work (int noutput_items,
		gr_vector_int &ninput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items)
{
	//cout << "sync" << endl;
	const int *in = (const int*) input_items[0];
	int *out = (int *) output_items[0];
	int written = 0;

	if (!do_synchronize) {
		if (tag_exists_in_range("response_beginning", ninput_items[0])) {
			do_synchronize = true;
			previous_zc = 0;
			current_zc = 0;
			previous_zc_absolute = 0;
			//cout << "Response start tag received at synchronizer" << endl;
		}
	}
	int absolute_offset = nitems_read(0);

	double pivot = 7;
	if (do_synchronize) {
		if (tag_exists_in_range("response_ending", ninput_items[0])) {
			do_synchronize = false;
			//cout << "Response end tag received at synchronizer" << endl;
		}
		previous_zc = previous_zc_absolute - (double)absolute_offset;
		for (int i = 0; i < ninput_items[0]-1; i++) {
			if(in[i]!=in[i+1]){ // If there is a zero crossing
				current_zc = i;
				out[written++]=in[i];

				if (current_zc-previous_zc>pivot) { // We have found a long symbol in the Miller encoding, so add another bit
					out[written++]=in[i];
				}
				previous_zc = current_zc;
			}
		}
		previous_zc_absolute = previous_zc + absolute_offset;
	}

	propagate_tags(ninput_items[0], written);
	consume_each(ninput_items[0]-1);
	return written;
}

void
Synchronizer_impl::propagate_tags(int ninput_items, int written){
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items);

	BOOST_FOREACH(gr::tag_t &tag, tags){
		tag.offset = nitems_written(0);
		this->add_item_tag(0, tag);
	}
}

bool
Synchronizer_impl::tag_exists_in_range(std::string key, int ninput_items){
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items, pmt::string_to_symbol(key));
	if (!tags.empty()) {
		//cout << "Tag found at " << this->name() << ": " << pmt::symbol_to_string(tags[0].key) << endl;
	}
	return !tags.empty();
}

void
Synchronizer_impl::update_blf(pmt::pmt_t msg)
{
	double blf = pmt::to_double(msg);
	double cycle_length = (d_sampling_rate/blf);
	d_pivot = (3/4)*cycle_length; //The pivot is simply the middle point between a miller short pulse and a miller long pulse: 3/4 cycle = (1/2 cycle + 1 cycle)/2
}
} /* namespace RFID_MAC */
} /* namespace gr */
