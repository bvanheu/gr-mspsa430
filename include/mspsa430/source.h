/* -*- c++ -*- */
/*
 * Copyright 2013 <+YOU OR YOUR COMPANY+>.
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


#ifndef INCLUDED_MSPSA430_SOURCE_H
#define INCLUDED_MSPSA430_SOURCE_H

#include <mspsa430/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace mspsa430 {

    /*!
     * \brief <+description of block+>
     * \ingroup mspsa430
     *
     */
    class MSPSA430_API source : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<source> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of mspsa430::source.
       *
       * To avoid accidental use of raw pointers, mspsa430::source's
       * constructor is in a private implementation
       * class. mspsa430::source::make is the public interface for
       * creating new instances.
       */
      static sptr make(const std::string path);
    };

  } // namespace mspsa430
} // namespace gr

#endif /* INCLUDED_MSPSA430_SOURCE_H */

