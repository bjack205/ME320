#!/bin/bash

DIR="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
cd $DIR

# ------------------------
# Install Python 3 and pip
# ------------------------

if ! type pip3 > /dev/null; then
	if [[ "$OSTYPE" == "linux-gnu" ]]; then
		sudo apt-get install python3-pip
	elif [[ "$OSTYPE" == "darwin"* ]]; then
		brew install python3
	fi
fi

# ----------------
# Setup virtualenv
# ----------------

pip3 install virtualenv
virtualenv -p python3 env
source env/bin/activate
pip install -r requirements.txt
