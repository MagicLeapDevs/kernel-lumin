#!/bin/sh

if [ ! -f "$1" ]; then
	if [ "$#" -eq 3 ]; then
		SEED=$3
	else
		SEED=`od -A n -t x8 -N 32 /dev/urandom | tr -d ' \n'`
	fi
	echo "const char *randstruct_seed = \"$SEED\";" > "$1"
	HASH=`echo -n "$SEED" | sha256sum | cut -d" " -f1 | tr -d ' \n'`
	echo "#define RANDSTRUCT_HASHED_SEED \"$HASH\"" > "$2"
fi
