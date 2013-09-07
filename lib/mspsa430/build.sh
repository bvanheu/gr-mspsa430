#!/bin/bash

clang -shared mspsa430_lld.c -o libmspsa430_lld.so
clang++ mspsa430.cpp -L . -l mspsa430_lld -o mspsa430

