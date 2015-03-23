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


#ifndef INCLUDED_RFID_MAC_NORMALIZER_H
#define INCLUDED_RFID_MAC_NORMALIZER_H

#include "api.h"
#include <gnuradio/block.h>

namespace gr {
  namespace RFID_MAC {

    /*!
     * \brief <+description of block+>
     * \ingroup RFID_MAC
     *
     */
    class RFID_MAC_API Normalizer : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<Normalizer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of RFID_MAC::Stream_Centering.
       *
       * To avoid accidental use of raw pointers, RFID_MAC::Stream_Centering's
       * constructor is in a private implementation
       * class. RFID_MAC::Stream_Centering::make is the public interface for
       * creating new instances.
       */
      static sptr make(int sampling_rate);
    };

  } // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_NORMALIZER_H */

