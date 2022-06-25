#!/bin/sh

PRJ_ROOT=`git rev-parse --show-toplevel`
VERSION=`git rev-parse --short HEAD`

clang-format --style=Google -i ${PRJ_ROOT}/src/* *.ino *.cpp *.h ${PRJ_ROOT}/examples/*/*.ino ${PRJ_ROOT}/examples/*/test_seq*
echo ${VERSION}
sed -i -e 's/#define VERSION.*$$/#define VERSION "post-$(VERSION)"/' ${PRJ_ROOT}/examples/StepperDemo/StepperDemo.ino

