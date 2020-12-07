#/bin/bash

# Copyright (c) 2019 Cable Television Laboratories, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions, and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The names of the authors may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# Alternatively, provided that this notice is retained in full, this
# software may be distributed under the terms of the GNU General
# Public License ("GPL") version 2, in which case the provisions of the
# GPL apply INSTEAD OF those given above.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# This script uses pdfjam to run the python plotting programs and merge them
# into a single document.  The first argument is the filename prefix.
# example usage:  ./plot.sh dctcp
# example output: dctcp.pdf

if [ $# -eq 0 ]
then
    echo "No arguments supplied, exiting"
    exit 1
fi
python plot-ping-rtt.py ping-rtt.dat
python plot-dctcp-alpha.py dctcp-alpha.dat
python plot-tcp-throughput.py tcp-throughput.dat
python plot-queue-length.py queue-length.dat
python plot-queue-marks-frequency.py queue-marks-frequency.dat
python plot-tcp-cwnd.py tcp-cwnd.dat
python plot-tcp-rtt.py tcp-rtt.dat
# Select plots for single-page representation
pdfjam tcp-throughput.pdf tcp-cwnd.pdf dctcp-alpha.pdf queue-depth.pdf queue-marks-frequency.pdf ping-rtt.pdf --nup 2x3 --outfile $1.pdf
