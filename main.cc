/*
 * main.cc
 *
 *  Created on: Dec 2, 2014
 *      Author: master
 */
#include <gnuradio/top_block.h>
#include <gnuradio/uhd/usrp_source.h>
#include <gnuradio/uhd/usrp_sink.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/complex_to_mag.h>
#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/filter/fir_filter_ccc.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/tag_debug.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp> //sleep
#include <boost/program_options.hpp>
#include <csignal>
#include <iostream>

#include "options.h"
#include "MAC_impl.h"
#include "Encoder_impl.h"
#include "Gatekeeper_impl.h"
#include "Normalizer_impl.h"
#include "Synchronizer_impl.h"
#include "Decoder_impl.h"

#include <fstream>
#include <iostream>
using namespace std;

using namespace gr::RFID_MAC;
using namespace std;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/***********************************************************************
 * Main w/ program options
 **********************************************************************/
int main(int argc, char *argv[]){
	string device_addr ="";

	//top block
	gr::top_block_sptr tb = gr::make_top_block("RFID Reader");

	/* ===============================================================
	 * Filter settings
	 * These are straight from the original gen2reader since we do not
	 * understand the numbers in the num_taps calculation,
	 * and we don't know enough about filters to redesign it.
	 * ===============================================================*/
	int dec_rate = 16;
	int num_taps = int(64000 / ( (dec_rate * 4) * 256 ));
	gr_complex comp_one = gr_complex(1,1);
	std::vector<gr_complex> taps;
	for (int i = 0; i < num_taps; i++) {
		taps.push_back(comp_one);
	}

	//--------------------------------------------------------------------------------
	//-- custom blocks
	//--------------------------------------------------------------------------------
	//receive chain
	boost::shared_ptr<Gatekeeper_impl> gatekeeper = boost::make_shared<Gatekeeper_impl>(Options::get_receive_chain_sampling_rate());
	boost::shared_ptr<Normalizer_impl> normalizer = boost::make_shared<Normalizer_impl>(Options::get_receive_chain_sampling_rate());
	boost::shared_ptr<Synchronizer_impl> synchronizer = boost::make_shared<Synchronizer_impl>(Options::get_receive_chain_sampling_rate());
	boost::shared_ptr<Decoder_impl> decoder = boost::make_shared<Decoder_impl>();

	//transmit chain
	boost::shared_ptr<MAC_impl> reader = boost::make_shared<MAC_impl>();
	boost::shared_ptr<Encoder_impl> encoder= boost::make_shared<Encoder_impl>();

	//--------------------------------------------------------------------------------
	//-- standard blocks
	//--------------------------------------------------------------------------------
	gr::filter::fir_filter_ccc::sptr matched_filt = gr::filter::fir_filter_ccc::make(Options::fir_decimation, taps);
	gr::blocks::float_to_complex::sptr f_to_c = gr::blocks::float_to_complex::make();
	gr::blocks::complex_to_mag::sptr c_to_m = gr::blocks::complex_to_mag::make();

	// usrp source block
	gr::uhd::usrp_source::sptr usrp_source = gr::uhd::usrp_source::make
			(device_addr, uhd::stream_args_t("fc32"));
	usrp_source->set_samp_rate(Options::USRP_SOURCE_SAMPLING_RATE); //set sample rate
	usrp_source->set_center_freq(Options::USRP_CENTER_FREQUENCY); // set center frequency

	// usrp_sink block
	uhd::stream_args_t s_args = uhd::stream_args_t("fc32");
	s_args.channels = vector<size_t>(1); //set number of channels
	gr::uhd::usrp_sink::sptr usrp_sink = gr::uhd::usrp_sink::make(device_addr, s_args, "");
	usrp_sink->set_samp_rate(Options::USRP_SINK_SAMPLING_RATE);
	usrp_sink->set_center_freq(Options::USRP_CENTER_FREQUENCY);
	usrp_sink->set_gain(Options::USRP_SINK_GAIN, size_t(0));

	//file sinks
	gr::blocks::file_sink::sptr float_sink_1 = gr::blocks::file_sink::make(sizeof(float), "/home/master/Documents/float_1.out", false);
	gr::blocks::file_sink::sptr float_sink_2 = gr::blocks::file_sink::make(sizeof(float), "/home/master/Documents/float_2.out", false);
	gr::blocks::file_sink::sptr complex_sink_1 = gr::blocks::file_sink::make(sizeof(complex<float>), "/home/master/Documents/complex_1.out", false);
	gr::blocks::file_sink::sptr float_sink_3 = gr::blocks::file_sink::make(sizeof(int), "/home/master/Documents/float_3.out", false);

	//file sources
	gr::blocks::file_source::sptr gatekeeper_test_filesource = gr::blocks::file_source::make(sizeof(float), "/home/master/Documents/gatekeeper_test_mag.in", false);

	//tag debuggers
	gr::blocks::tag_debug::sptr tag_debugger = gr::blocks::tag_debug::make(sizeof(float), "td_1");
	gr::blocks::tag_debug::sptr tag_debugger_2 = gr::blocks::tag_debug::make(sizeof(float), "td_2");

	//------------------------------------------------------------------
	//-- connect the block message ports
	//------------------------------------------------------------------
	tb->msg_connect(reader, pmt::mp("reset_gate_message_port") , gatekeeper, pmt::mp("reset_gate_message_port"));
	tb->msg_connect(reader, pmt::mp("update_blf_message_port") , gatekeeper, pmt::mp("update_blf_message_port"));
	tb->msg_connect(reader, pmt::mp("update_t1_message_port") , gatekeeper, pmt::mp("update_t1_message_port"));

	tb->msg_connect(reader, pmt::mp("update_blf_message_port") , normalizer, pmt::mp("update_blf_message_port"));

	tb->msg_connect(reader, pmt::mp("update_blf_message_port") , synchronizer, pmt::mp("update_blf_message_port"));

	tb->msg_connect(reader, pmt::mp("update_miller_message_port") , decoder, pmt::mp("update_miller_message_port"));

	tb->msg_connect(gatekeeper, pmt::mp("gatekeeper_failure_port_out") , reader, pmt::mp("control_port_in"));
	tb->msg_connect(decoder, pmt::mp("decoder_message_out") , reader, pmt::mp("control_port_in"));

	//------------------------------------------------------------------
	//-- connect the block streams
	//------------------------------------------------------------------
	//receive chain connections
	tb->connect(usrp_source, 0, matched_filt, 0);
	tb->connect(matched_filt, 0, c_to_m, 0);
	tb->connect(c_to_m, 0, gatekeeper, 0);
	tb->connect(gatekeeper, 0, normalizer, 0);
	tb->connect(normalizer, 0, synchronizer, 0);
	tb->connect(synchronizer, 0, decoder, 0);

	//transmit chain connections
	tb->connect(reader, 0, encoder, 0);
	tb->connect(encoder, 0, f_to_c, 0);
	tb->connect(f_to_c, 0, usrp_sink, 0);

	//file sink connections
	//tb->connect(matched_filt, 0, complex_sink_1, 0);
	tb->connect(c_to_m, 0, float_sink_2, 0);
	//tb->connect(gatekeeper, 1, float_sink_1, 0);

	//tag debugger connnections
	//tb->connect(gatekeeper, 0, tag_debugger, 0);
	//tb->connect(normalizer, 0, tag_debugger_2, 0);

	//------------------------------------------------------------------
	//-- start flow graph execution
	//------------------------------------------------------------------
	std::cout << "starting flow graph" << std::endl;
	std::cout << "press ctrl + c to exit" << std::endl;
	tb->start();

	//------------------------------------------------------------------
	//-- poll the exit signal while running
	//------------------------------------------------------------------
	std::signal(SIGINT, &sig_int_handler);
	while (not stop_signal_called){
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}

	//------------------------------------------------------------------
	//-- stop flow graph execution
	//------------------------------------------------------------------
	std::cout << "stopping flow graph" << std::endl;
	tb->stop();
	tb->wait();
	std::cout << "done!" << std::endl;
	return 0;
}


