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


#ifndef INCLUDED_RFID_MAC_ENCODER_H
#define INCLUDED_RFID_MAC_ENCODER_H

#include "api.h"
#include <gnuradio/block.h>

namespace gr {
  namespace RFID_MAC {

    /*!
     * \brief <+description of block+>
     * \ingroup RFID_MAC
     *
     */
    class RFID_MAC_API Encoder : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<Encoder> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of RFID_MAC::PIE_Encoding.
       *
       * To avoid accidental use of raw pointers, RFID_MAC::PIE_Encoding's
       * constructor is in a private implementation
       * class. RFID_MAC::PIE_Encoding::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace RFID_MAC
} // namespace gr

#endif /* INCLUDED_RFID_MAC_ENCODER_H */

