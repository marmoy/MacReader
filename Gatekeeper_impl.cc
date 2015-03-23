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
#include "Gatekeeper_impl.h"

namespace gr {
namespace RFID_MAC {

Gatekeeper::sptr
Gatekeeper::make(int sampling_rate)
{
	return gnuradio::get_initial_sptr(new Gatekeeper_impl(sampling_rate));
}

/*
 * The private constructor
 */
Gatekeeper_impl::Gatekeeper_impl(int sampling_rate)
: gr::block("Gatekeeper",
		gr::io_signature::make(1, 1, sizeof(float)),
		gr::io_signature::make(1, 1, sizeof(float)))
{
	d_sample_rate = sampling_rate; //Ensure that this is the actual sample rate on the input port
	USRP_SIGNAL_SETTLING_LENGTH = ceil(sampling_rate/(double)1e5);
	tag_dict = pmt::make_dict();

	message_port_register_out(gatekeeper_failure_port_out);
	message_port_register_in(reset_gate_message_port);
	set_msg_handler(reset_gate_message_port, boost::bind(&Gatekeeper_impl::reset_gate, this, _1));
	message_port_register_in(update_blf_message_port);
	set_msg_handler(update_blf_message_port, boost::bind(&Gatekeeper_impl::update_blf, this, _1));
	message_port_register_in(update_t1_message_port);
	set_msg_handler(update_t1_message_port, boost::bind(&Gatekeeper_impl::update_t1, this, _1));
}

/*
 * Our virtual destructor.
 */
Gatekeeper_impl::~Gatekeeper_impl()
{}

void //Whatever we set ninput_items_required to, it only guarantees a minimum. The actual number may be larger
Gatekeeper_impl::forecast(int noutput_items,
		gr_vector_int &ninput_items_required)
{
	unsigned ninputs = ninput_items_required.size ();
	for(unsigned i = 0; i < ninputs; i++){
		ninput_items_required[i] = min_nitems_required;
	}
}

int
Gatekeeper_impl::general_work(int noutput_items,
		gr_vector_int &ninput_items, //array containing the number of input items for each port of the block. Index 0 corresponds to port 0, 1 to 1 and so on.
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items)
{
	//cout << "gate" << endl;
	const float *in = (const float *) input_items[0];
	float *out = (float *) output_items[0];
	//float *out_debug = (float *) output_items[1];
	int consumed = ninput_items[0];
	int written = 0;

	switch (state) {
	case FIND_CW_RISING_EDGE:
	{
		for (int i = 0; i<ninput_items[0]-50; i+=50) {
			if (in[i]>1&&in[i+50]>1) { // Precision is not important, so we only check first and last item.
				consumed = i + 50;
				advance_state(FIND_COMMAND_RISING_EDGE);
				break;
			}
		}
		break;
	}
	case FIND_COMMAND_RISING_EDGE:
	{
		for (int i = 0; i<ninput_items[0]-1; i++) {
			if (fabs(in[i]-in[i+1])>COMMAND_EDGE_SLOPE) {
				consumed = i;
				command_start_index = this->nitems_read(0)+i+USRP_SIGNAL_SETTLING_LENGTH;
				inject_tag("t2_length", command_start_index - response_end_index, this->nitems_written(0));
				t1_start_index = command_start_index + command_length;
				skip_to_index = t1_start_index + 2*USRP_SIGNAL_SETTLING_LENGTH;
				advance_state(SKIP_COMMAND);
				break;
			}
		}
		break;
	}
	case SKIP_COMMAND:
	{
		if (this->nitems_read(0)+ninput_items[0]>=skip_to_index) { // If we have reached the right window
			consumed = skip_to_index - this->nitems_read(0); // Skip the remaining few items
			advance_state(FIND_RESPONSE_RISING_EDGE);
		}
		else {
			consumed = ninput_items[0]; // Skip the whole window
		}
		break;
	}
	case FIND_RESPONSE_RISING_EDGE:
	{
		int t1_count = this->nitems_read(0) - t1_start_index;
		for (int i = 1; i<ninput_items[0]; i++) {
			if (t1_count==2*USRP_SIGNAL_SETTLING_LENGTH) {
				float mean;
				volk_32f_stddev_and_mean_32f_x2(&std_dev_noise, &mean, &in[i], slice_length);
			}
			if (t1_count++>T1_MAX_LENGTH) {
				consumed = i;
				handle_failure(ERRORS::LONG_T1);
				break;
			}
			if (fabs(in[i]-in[i-1])>TAG_RESPONSE_EDGE_SLOPE) {
				consumed = i;
				response_start_index = this->nitems_read(0)+i;
				inject_tag("response_beginning", response_start_index, this->nitems_written(0)+i);
				inject_tag("t1_length", t1_count, this->nitems_written(0)+i);
				advance_state(FIND_RESPONSE_FALLING_EDGE);
				break;
			}
		}
		break;
	}
	case FIND_RESPONSE_FALLING_EDGE:
	{	//TODO: use absolute offset to skip 90% of response
		if (abs(nitems_read(0)-response_start_index)<5) {
			float mean;
			volk_32f_stddev_and_mean_32f_x2(&std_dev_signal, &mean, &in[ninput_items[0]/2], slice_length);
			cout << "SNR: " << 20*log10(std_dev_signal/std_dev_noise) << " dB"<< endl;
		}
		for (int i = 1; i < ninput_items[0]-3; i++) {
			if (fabs(in[i-1]-in[i])>TAG_RESPONSE_EDGE_SLOPE) { // As long as we keep finding  steep edges, we reset the count
				samples_without_change=0;
			}
			else {
				samples_without_change++;
			}
			if (samples_without_change>=slice_length) { //If we've counted a sufficient number of samples without steep edges, we know we have reached the end of the signal
				consumed = i-slice_length;
				response_end_index = this->nitems_read(0)+i-slice_length;
				int response_length = response_end_index - response_start_index;
				inject_tag("response_ending_over_shoot", ninput_items[0]-consumed, 0);
				inject_tag("response_ending", response_end_index, 0);
				inject_tag("response_length", response_length, 0);
				samples_without_change = 0;
				advance_state(WAIT);
				break;
			}
		}
		for (int i = 0; i < consumed; i++) {
			out[written++] = in[i];
		}
		break;
	}
	case WAIT:	// Waiting for the transmit chain to catch up, see reset_gate()
	{
		if (resume) {
			resume = false;
			advance_state(FIND_COMMAND_RISING_EDGE);
		}
		consumed = 0;
		break;
	}
	default:
		break;
	}

	/*for (int i = 0; i < consumed; i++) {
		out_debug[i] = in[i];
	}

	if (consumed<ninput_items[0]) { //For debugging the output with markers. Might possibly be replaceable by tags and loading the file into a gnuradio plot sink
		out_debug[consumed-1]=10;
	}*/

	consume_each(consumed);
	return consumed;
}

void
Gatekeeper_impl::reset_gate(pmt::pmt_t command_us)
{
	command_length = microseconds_to_samples(pmt::to_uint64(command_us), d_sample_rate);
	assert(pmt::to_uint64(command_us)!=0);
	resume = true;
}

void
Gatekeeper_impl::update_blf(pmt::pmt_t blf)
{
	slice_length = ceil(Options::min_window_size_cycles*(d_sample_rate/pmt::to_double(blf)));
}

void
Gatekeeper_impl::update_t1(pmt::pmt_t t1_nominal_us)
{
	T1_MAX_LENGTH = 2*microseconds_to_samples(pmt::to_double(t1_nominal_us), d_sample_rate); // We set the max t1 to double t1 nominal, to accommodate the WISP.
}

void
Gatekeeper_impl::handle_failure(int error)
{
	inject_tag("failure", error, this->nitems_read(0));
	message_port_pub(gatekeeper_failure_port_out, tag_dict);
	tag_dict = pmt::make_dict(); //Reset dict
	advance_state(WAIT);
}

void
Gatekeeper_impl::advance_state(int next_state){
	state = next_state;
	//cout << "Gate state: " << next_state << endl;
}
} /* namespace RFID_MAC */
} /* namespace gr */

