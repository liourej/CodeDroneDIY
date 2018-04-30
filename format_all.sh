#! /bin/bash

find ./src/ \
	-type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cc" -o -name \
	"*.cxx" -o -name "*.c" -o -name "*.h" \) \
| while read -r fname; do
	clang-format "$fname" -style=file -i
done
