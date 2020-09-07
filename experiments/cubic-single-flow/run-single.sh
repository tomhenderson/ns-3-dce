#!/bin/bash

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

#
# Basic configuration to replicate scenarios from Pete Heist's results:
# scenario 2 (issue 16):  controlScenario=1, m3QueueType=fq, ...  
# scenario 3 (issue 16):  controlScenario=1, m3QueueType=codel, ...  
# scenario 5 (issue 17):  controlScenario=0, m3QueueType=fq, link3rate=52.5Mbps, link5RateRatio=0.9524, ...  
# scenario 6 (issue 17):  controlScenario=0, m3QueueType=fq, ...

####################
# scenario details 
scenario_id=one-flow
link3rate=50Mbps
link5rateRatio=1
rtt=50ms
controlScenario=1
enablePcap=0
firstTcpType=reno
m3QueueType=codel
queueUseEcn=1
dceSender=1
dceReceiver=1
RngRun=1
####################

pathToTopLevelDir="../.."
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/${pathToTopLevelDir}/build/lib
export saveDatFiles=true

dirname=$1
if [ -z ${dirname} ]
then
	dirname=${firstTcpType}-${dceSender}-${queueUseEcn}
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
	cp ${EXECUTABLE} ${resultsDir}/cubic-one-flow
else
	echo "$EXECUTABLE not found, exiting"
	exit 1
fi
cp $0 ${resultsDir}/.
cp *.py ${resultsDir}/.
cp plot.sh ${resultsDir}/.
cp ${pathToTopLevelDir}/example/tcp-validation.cc ${resultsDir}/.
cd ${resultsDir}

if [ $controlScenario == 1 ]; then
  echo starting single bottleneck scenario $scenario_id: ${firstTcpType} ${m3QueueType} ${link3rate} ${link5rateRatio} ${rtt}
else
  echo starting two bottleneck scenario $scenario_id: ${firstTcpType} ${m3QueueType} ${link3rate} ${link5rateRatio} ${rtt}
fi

pingTraceFile=ping-rtt.dat
tcpRttTraceFile=tcp-rtt.dat
tcpCwndTraceFile=tcp-cwnd.dat
tcpThroughputTraceFile=tcp-throughput.dat
m1DropTraceFile=m1-drops.dat
m1DropsFrequencyTraceFile=m1-drops-frequency.dat
m1LengthTraceFile=m1-length.dat
m3MarkTraceFile=m3-marks.dat
m3MarksFrequencyTraceFile=m3-marks-frequency.dat
m3DropTraceFile=m3-drops.dat
m3LengthTraceFile=m3-length.dat

cmd="./cubic-one-flow \
    --controlScenario=$controlScenario \
    --firstTcpType=$firstTcpType \
    --m3QueueType=$m3QueueType \
    --link3rate=$link3rate \
    --link5rateRatio=$link5rateRatio \
    --baseRtt=$rtt \
    --dceSender=$dceSender \
    --dceReceiver=$dceReceiver \
    --queueUseEcn=$queueUseEcn \
    --enablePcap=$enablePcap \
    --pingTraceFile=$pingTraceFile \
    --firstTcpRttTraceFile=$tcpRttTraceFile \
    --firstTcpCwndTraceFile=$tcpCwndTraceFile \
    --firstTcpThroughputTraceFile=$tcpThroughputTraceFile \
    --m1DropTraceFile=$m1DropTraceFile \
    --m1DropsFrequencyTraceFile=$m1DropsFrequencyTraceFile \
    --m1LengthTraceFile=$m1LengthTraceFile \
    --m3MarkTraceFile=$m3MarkTraceFile \
    --m3MarksFrequencyTraceFile=$m3MarksFrequencyTraceFile \
    --m3DropTraceFile=$m3DropTraceFile \
    --m3LengthTraceFile=$m3LengthTraceFile \
    --RngRun=${RngRun}"
echo $cmd
echo $cmd > log.txt
`$cmd`

if [ -e files-1/var/log/messages ]
then
	cat files-1/var/log/messages | awk '{ print $2, $3, $4, $5}' | grep cwnd | awk '{ print ($1 - 300)"."$2, $4, $5}' | uniq > tcp-cwnd.dat
fi

if [ -e tsvwg-scenarios-1-0.pcap ]
then
	mv tsvwg-scenarios-1-0.pcap sender.pcap
	mv tsvwg-scenarios-9-0.pcap receiver.pcap
	rm -rf tsvwg-scenarios-*.pcap
fi

if [ "$dceSender" -eq "1" ]
then
	if [ "$queueUseEcn" -eq "1" ]
	then
		fname="${firstTcpType}-${m3QueueType}-dce-ecn-${link3rate}-${rtt}"
	else
		fname="${firstTcpType}-${m3QueueType}-dce-noecn-${link3rate}-${rtt}"
	fi
else
	if [ "$queueUseEcn" -eq "1" ]
	then
		fname="${firstTcpType}-${m3QueueType}-ns3-ecn-${link3rate}-${rtt}"
	else
		fname="${firstTcpType}-${m3QueueType}-ns3-noecn-${link3rate}-${rtt}"
	fi
fi

./plot.sh $fname >>log.txt 2>&1

if ! $saveDatFiles
then
	rm -rf ${scenario_id}
fi

echo finished scenario $dirname

exit
