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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "source_impl.h"

namespace gr {
  namespace mspsa430 {

    source::sptr source::make(const std::string path)
    {
      return gnuradio::get_initial_sptr(new source_impl(path));
    }

    static const int MIN_IN = 0;  /*!< Mininum number of input streams. */
    static const int MAX_IN = 0;  /*!< Maximum number of input streams. */
    static const int MIN_OUT = 1; /*!< Minimum number of output streams. */
    static const int MAX_OUT = 1; /*!< Maximum number of output streams. */

    /*
     * The private constructor
     *
     * Makes a source block
     */
    source_impl::source_impl(const std::string path)
      : gr::sync_block("mspsa430_source",
              gr::io_signature::make(MIN_IN, MAX_IN, sizeof(uint8_t)),
              gr::io_signature::make(MIN_OUT, MAX_OUT, sizeof(uint8_t)))
    {
        this->m = new mspsa430(&this->lld);
        this->m->connect("/dev/ttyACM0", 921600);
        this->m->setup();
    }

    /*
     * Our virtual destructor.
     */
    source_impl::~source_impl()
    {
        this->m->disconnect();
    }

    int
    source_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        //const int8_t *in = (const int8_t *) input_items[0];
        int8_t *out = (int8_t *) output_items[0];
        size_t i = 0;

        // Do <+signal processing+>
        std::cout << "Nbr output items: " << noutput_items << std::endl;
        std::vector<int8_t> spectrum;

        spectrum = this->m->get_spectrum_no_init();

        for (i=0; i<spectrum.size(); i++) {
            out[i] = spectrum.at(i);
        }

        // Tell runtime system how many output items we produced.
        //return noutput_items;
        return i;
    }

  } /* namespace mspsa430 */
} /* namespace gr */

