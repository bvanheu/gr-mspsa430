#!/bin/bash

# uncomment main() in mspsa430.cpp
# sudo ./mspsa430
CC=c++
$CC mspsa430_lld.cpp mspsa430.cpp -o mspsa430 -std=c++11
