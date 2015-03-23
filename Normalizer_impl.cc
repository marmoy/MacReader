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
#include <volk/volk.h>

#include "Normalizer_impl.h"
#include "options.h"

namespace gr {
namespace RFID_MAC {

Normalizer::sptr
Normalizer::make(int sampling_rate)
{
	return gnuradio::get_initial_sptr(new Normalizer_impl(sampling_rate));
}

/*
 * The private constructor
 */
Normalizer_impl::Normalizer_impl(int sampling_rate)
: gr::block("Normalizer",
		gr::io_signature::make(1, 1, sizeof(float)),
		gr::io_signature::make(1, 1, sizeof(int)))
{
	d_sampling_rate = sampling_rate;
	set_tag_propagation_policy(tag_propagation_policy_t::TPP_DONT);
	message_port_register_in(update_blf_message_port);
	set_msg_handler(update_blf_message_port, boost::bind(&Normalizer_impl::update_blf, this, _1));
}

/*
 * Our virtual destructor.
 */
Normalizer_impl::~Normalizer_impl(){}

int
Normalizer_impl::general_work(int noutput_items, gr_vector_int &ninput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items)
{
	//cout << "center" << endl;
	const float *in = (const float *) input_items[0];
	int *out = (int *) output_items[0];
	int written = 0;
	int consumed = ninput_items[0];

	if (!do_normalize) {
		if (tag_exists_in_range("response_beginning", ninput_items[0])) {
			do_normalize = true;
		}
	}
	if (do_normalize) {
		for (int stream_element = floor(d_window_size/2); stream_element < ninput_items[0]-ceil(d_window_size/2); stream_element++) {
			float mean, std_dev;
			volk_32f_stddev_and_mean_32f_x2(&std_dev, &mean, &in[(int)(stream_element-floor(d_window_size/2))], d_window_size); // Calculate mean over d_window_size with stream_element in the middle.

			if (in[stream_element]>mean) {
				out[written++]=1;
				//cout << 1;
			}
			else {
				out[written++]=-1;
				//cout << 0;
			}
		}
		consumed = ninput_items[0]-d_window_size;
		if (tag_exists_in_range("response_ending", ninput_items[0])) {
			do_normalize = false;
			float mean, std_dev;
			volk_32f_stddev_and_mean_32f_x2(&std_dev, &mean, &in[(int)(ninput_items[0]-ceil(d_window_size/2))], ceil(d_window_size/2)); // Calculate mean over d_window_size with stream_element in the middle.
			for (int i = ninput_items[0]-ceil(d_window_size/2); i < ninput_items[0]; i++) {
				if (in[i]>mean) {
					out[written++]=1;
					//cout << 1;
				}
				else {
					out[written++]=-1;
					//cout << 0;
				}
			}
			consumed = ninput_items[0];
		}
	}

	propagate_tags(ninput_items[0], written);
	consume_each(consumed);
	return written;
}

void //Whatever we set ninput_items_required to, it only guarantees a minimum. The actual number may be larger
Normalizer_impl::forecast(int noutput_items,
		gr_vector_int &ninput_items_required)
{
	unsigned ninputs = ninput_items_required.size ();
	for(unsigned i = 0; i < ninputs; i++)
	{
		ninput_items_required[i] = 2*d_window_size;
	}
}

void
Normalizer_impl::propagate_tags(int ninput_items, int written){
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items);

	BOOST_FOREACH(gr::tag_t &tag, tags){
		tag.offset = nitems_written(0);
		this->add_item_tag(0, tag);
	}
}

bool
Normalizer_impl::tag_exists_in_range(std::string key, int ninput_items){
	const uint64_t samp0_count = this->nitems_read(0);
	std::vector<gr::tag_t> tags;
	get_tags_in_range(tags, 0, samp0_count, samp0_count + ninput_items, pmt::string_to_symbol(key));
	if (!tags.empty()) {
		//cout << "tag found at " << this->name() << ": " << pmt::symbol_to_string(tags[0].key) << endl;
	}
	return !tags.empty();
}

void
Normalizer_impl::update_blf(pmt::pmt_t blf)
{
	d_window_size = ceil(Options::min_window_size_cycles*(d_sampling_rate/pmt::to_double(blf)));// This is carefully chosen to match 1.5 BLF cycles, in order to maximize std_dev, but minimize effect from noise and falloff
}
} /* namespace RFID_MAC */
} /* namespace gr */

