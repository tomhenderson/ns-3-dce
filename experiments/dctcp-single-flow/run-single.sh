#!/bin/bash

# Copyright (c) 2019 Cable Television Laboratories, Inc.
# Copyright (c) 2020 Tom Henderson (adapted for DCTCP testing)
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

####################
# scenario details 
scenario_id=one-flow
linkRate=50Mbps
rtt=80ms
enablePcap=0
firstTcpType=dctcp
queueType=codel
queueUseEcn=1
dceSender=0
dceReceiver=0
delAckCount=2
stopTime=60s
RngRun=1
####################

pathToTopLevelDir="../.."
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/${pathToTopLevelDir}/build/lib

dirname=$1
if [ -z ${dirname} ]
then
	dirname=${firstTcpType}-${dceSender}-${dceReceiver}-${queueUseEcn}
fi

./waf build > /dev/null 2>&1
if [ $? -ne 0 ]; then
	echo "Waf build faild"
	exit 1
fi 
resultsDir=results/$dirname-`date +%Y%m%d-%H%M%S`
mkdir -p ${resultsDir}
repositoryVersion=`git rev-parse --abbrev-ref HEAD`
repositoryVersion+=' commit '
repositoryVersion+=`git rev-parse --short HEAD`
repositoryVersion+=' '
repositoryVersion+=`git log -1 --format=%cd`
echo $repositoryVersion > ${resultsDir}/version.txt
gitDiff=`git diff`
if [[ $gitDiff ]]
then
	echo "$gitDiff" >> ${resultsDir}/version.txt
fi
EXECUTABLE_NAME=tcp-validation
EXECUTABLE=${pathToTopLevelDir}/build/bin/${EXECUTABLE_NAME}
if [ -f "$EXECUTABLE" ]; then
	cp ${EXECUTABLE} ${resultsDir}/dctcp-one-flow
else
	echo "$EXECUTABLE not found, exiting"
	exit 1
fi
cp $0 ${resultsDir}/.
cp *.py ${resultsDir}/.
cp plot.sh ${resultsDir}/.
cp ${pathToTopLevelDir}/example/tcp-validation.cc ${resultsDir}/.
cd ${resultsDir}

  echo starting single bottleneck scenario $scenario_id: ${firstTcpType} ${queueType} ${linkRate} ${rtt}

pingTraceFile=ping-rtt.dat
tcpRttTraceFile=tcp-rtt.dat
tcpCwndTraceFile=tcp-cwnd.dat
dctcpTraceFile=dctcp-alpha.dat
tcpThroughputTraceFile=tcp-throughput.dat
queueMarksFrequencyTraceFile=queue-marks-frequency.dat
queueLengthTraceFile=queue-length.dat

cmd="./dctcp-one-flow \
    --firstTcpType=$firstTcpType \
    --queueType=$queueType \
    --linkRate=$linkRate \
    --baseRtt=$rtt \
    --dceSender=$dceSender \
    --dceReceiver=$dceReceiver \
    --queueUseEcn=$queueUseEcn \
    --enablePcap=$enablePcap \
    --pingTraceFile=$pingTraceFile \
    --firstDctcpTraceFile=$dctcpTraceFile \
    --firstTcpRttTraceFile=$tcpRttTraceFile \
    --firstTcpCwndTraceFile=$tcpCwndTraceFile \
    --firstTcpThroughputTraceFile=$tcpThroughputTraceFile \
    --queueMarksFrequencyTraceFile=$queueMarksFrequencyTraceFile \
    --queueLengthTraceFile=$queueLengthTraceFile \
    --ns3::TcpSocket::DelAckCount=$delAckCount \
    --stopTime=$stopTime \
    --RngRun=${RngRun}"
echo $cmd
echo $cmd > log.out 2>&1
`$cmd`

# Parse DCE message file for cwnd and alpha stats
if [ -e files-1/var/log/messages ]
then
	cat files-1/var/log/messages | awk '{ print $2, $3, $4, $5}' | grep cwnd | awk '{ print ($1 - 300)"."$2, $4, $5}' | uniq > tcp-cwnd.dat
	cat files-1/var/log/messages | awk '{ print $2, $3, $4, $5}' | grep dctcp_alpha| awk '{ print ($1 - 300)"."$2, $4/1024}' | uniq > dctcp-alpha.dat
fi

if [ -e tcp-validation-1-0.pcap ]
then
	mv tcp-validation-1-0.pcap sender.pcap
	mv tcp-validation-6-0.pcap receiver.pcap
	rm -rf tcp-validation-*.pcap
fi

if [ "$dceSender" -eq "1" ]
then
	if [ "$queueUseEcn" -eq "1" ]
	then
		fname="${firstTcpType}-${queueType}-dce-ecn-${linkRate}-${rtt}"
	else
		fname="${firstTcpType}-${queueType}-dce-noecn-${linkRate}-${rtt}"
	fi
else
	if [ "$queueUseEcn" -eq "1" ]
	then
		fname="${firstTcpType}-${queueType}-ns3-ecn-${linkRate}-${rtt}"
	else
		fname="${firstTcpType}-${queueType}-ns3-noecn-${linkRate}-${rtt}"
	fi
fi

./plot.sh $fname >>log.out 2>&1

echo finished scenario $dirname

exit
