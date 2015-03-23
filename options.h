#ifndef INCLUDED_RFID_MAC_OPTIONS_H
#define INCLUDED_RFID_MAC_OPTIONS_H

namespace gr {
namespace RFID_MAC {

/* =====================================
 * ENUMS
 * =====================================*/
enum FRAMES {PREAMBLE, FRAMESYNC, NO_FRAME};
enum RESPONSES{RN16, EPC, NO_RESPONSE};
enum COMMANDS{QUERY, ACK, QREP, NAK, REQ_RN, READ, IDLE, FINAL};
enum LINK_TIMING{T1, T2, T3, T4};
enum LOG_LEVELS{ERROR, INFO};
enum BLOCKS{GATEKEEPER, NORMALIZER, SYNCHRONIZER, DECODER};
enum ERRORS{SHORT_T1, LONG_T1, SHORT_RESPONSE, LONG_RESPONSE, NO_PREAMBLE, INVALID_RESPONSE};
enum DIVIDE_RATIO {DR0, DR1};
enum MILLER_ENCODING {M1, M2, M4, M8};
enum TREXT {NO_PILOT_TONE, PILOT_TONE};
enum SELECT {ALL0, ALL1, NOT_SL, SL};
enum SESSION {S0, S1, S2, S3};
enum TARGET {A, B};

//TODO: rename options to something related to 'configuration'
class Options {
public:
	/* ===========================================================
	 * USRP configuration
	 * ONLY modify sink sampling rate if you understand the effects
	 * The current sampling rate corresponds to 1 sample per microsecond, which admittedly limits the resolution.
	 * If a higher resolution is needed, modify the encoder to use the time/sample conversion methods in this header.
	 * ===========================================================*/
	static constexpr double USRP_CENTER_FREQUENCY = 867.5e6;
	//static constexpr int USRP_SOURCE_SAMPLING_RATE = 781250; //40kHz
	static constexpr int USRP_SOURCE_SAMPLING_RATE = 5e6; //256kHz
	static constexpr int USRP_SINK_SAMPLING_RATE = 1e6;
	static constexpr int USRP_SINK_GAIN = 25; // Appropriate for monopole, quarterwave antennas. Better antennas may need less.

	/* ==========================================================
	 * Query command configuration - See section 6.3.2.11.2.1 and table 6.21 in the standard
	 * ==========================================================*/
	static const int miller_encoding_default = MILLER_ENCODING::M4;
	static const int trext_default = TREXT::NO_PILOT_TONE;
	static const int divide_ratio_default = DIVIDE_RATIO::DR1; // WARNING: TIGHTLY COUPLED WITH TRCAL!!!!!
	static const int sel_default = SELECT::ALL0;
	static const int session_default = SESSION::S0;
	static const int target_default = TARGET::A;
	static const int q_default = 0;

	/* ================================================================
	 * Filter configuration
	 * ================================================================*/
	static const int fir_decimation = 2;
	static int get_receive_chain_sampling_rate(){ return USRP_SOURCE_SAMPLING_RATE/(double)fir_decimation; }

	/* =====================================================
	 * Symbols (Data, Calibration, Framing). See sections 6.3.1.2.3 and 6.3.1.2.8 and figures 6.1 and 6.4 in the standard
	 * =====================================================*/
	static constexpr double TARI_us = 24; // Tari is the data symbol reference time, and defined as the length of a data0 symbol.
	static constexpr double DELIM_us = 12; // DELIM = Delimiter. Spec says 12.5 us ±5%.

	// 256kHz (WISP)
	static constexpr double TRcal_us = 83.3;
	// 40 kHz
	//static constexpr double TRcal_us = 200; // WARNING: TIGHTLY COUPLED WITH DIVIDE RATIO!!!!!

	static double get_TARI_us(){return TARI_us;}
	static double get_DELIM_us(){return DELIM_us;}
	static double get_TRcal_us(){return TRcal_us;}
	static double get_PW_us(){return 0.5*TARI_us;} // PW = Pulse Width. Spec says between MAX(0.265*Tari, 2) and 0.525*Tari
	static double get_DATA0_us(){return TARI_us;}
	static double get_DATA1_us(){return 2.0*TARI_us;} // Spec says between 1.5*Tari and 2*Tari
	static double get_RTcal_us(){ // RTcal = data0_width + data1_width
		double rtcal_us = get_DATA0_us() + get_DATA1_us();
		assert(1.1*rtcal_us<=TRcal_us&&TRcal_us<=3*rtcal_us);
		return rtcal_us;
	}
	static double get_PREAMBLE_us(){ return get_FRAME_SYNC_us() + get_TRcal_us();}
	static double get_FRAME_SYNC_us(){ return get_DELIM_us() + get_DATA0_us() + get_RTcal_us();	}
	static const int CW_HEAD_us = 1400;

	static constexpr double min_window_size_cycles = 2.5; //This number is based on the calculation of the smallest representative window of the tag response.

	static const int MAX_CYCLES = 5; //Limits the number of cycles to run, 0 is infinity
};
}
}
#endif

