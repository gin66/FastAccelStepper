#!/bin/sh

PRJ_ROOT=`git rev-parse --show-toplevel`
VERSION=`git rev-parse --short HEAD`

FILES=`find ${PRJ_ROOT} -path ${PRJ_ROOT}/extras/tests/simavr_based/simavr -prune -type f -name '*.ino' -or -type f -name '*.cpp' -or -type f -name '*.h'`
FILES_SRC=`find ${PRJ_ROOT}/src ${PRJ_ROOT}/examples -type f -a \( -name '*.ino' -o -name '*.cpp' -o -name '*.h' \) -a -not -name '*due*'`

echo ${FILES_SRC}

clang-format -style=file -i $FILES
cppcheck \
  --enable=style \
  --suppress=invalidPrintfArgType_sint \
  --suppress=unusedStructMember \
  --suppress=noConstructor \
  --suppress=ctuOneDefinitionRuleViolation \
  --suppress=unknownMacro \
  --suppress=knownConditionTrueFalse \
  -DPROGMEM= \
  --force \
  --check-level=exhaustive \
  --language=c++ \
  --std=c++11 \
  --addon=namingng.json \
  ${FILES_SRC}

echo ${VERSION}
sed -i -e 's/#define VERSION.*$$/#define VERSION "post-$(VERSION)"/' ${PRJ_ROOT}/examples/StepperDemo/StepperDemo.ino
