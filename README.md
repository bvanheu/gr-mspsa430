GR-MSPSA430
===========

This is an attempt to create a MSP-SA430-SUB1GHZ source block for GNU Radio.

How to build and install?
-------------------------
### Building
    $ mkdir build && cd build
    $ PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1 cmake ..

To install everything in */usr* intead of */usr/local*, run the following:

    $ PKG_CONFIG_ALLOW_SYSTEM_CLFAGS=1 cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr ..

### Installing
    $ sudo make install

Make sure that GNU Radio will find the locally installed block, if you installed locally.
In the file *~/.gnuradio/config.conf*, add the following:

    [grc]
    local_blocks_path=/usr/local/share/gnuradio/grc/blocks

