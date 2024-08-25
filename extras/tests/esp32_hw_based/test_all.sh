#!/bin/sh

# arduino idf4 based test
make compile
make M1.test M7.test

# idf4 based test
make compile_idf4
make M1.test M7.test

# idf5 only rmt-module supporting 8 steppers
make compile_idf5
make M1.test
