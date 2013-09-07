/* -*- c++ -*- */

#define MSPSA430_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "mspsa430_swig_doc.i"

%{
#include "mspsa430/source.h"
%}


%include "mspsa430/source.h"
GR_SWIG_BLOCK_MAGIC2(mspsa430, source);
