#!/bin/bash
if [[ ! -f README.md ]]; then
	cd ..
fi

find ./src -iname *.h -o -iname *.cpp -o -iname *.c | xargs clang-format --style=file -i
find ./vorago -iname *.h -o -iname *.cpp -o -iname *.c | xargs clang-format --style=file -i
