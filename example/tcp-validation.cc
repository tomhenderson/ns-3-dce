/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Cable Television Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the authors may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// tsvwg-scenarios.cc responds to issues 16 and 17 from the IETF Transport
// Area Working Group (tsvwg) tracker, with a simulation scenario aligned
// with scenarios 5 and 6 tested on Pete Heist's testbed.
//
// ---> downstream (primary data transfer from servers to clients)
// <--- upstream (return acks and ICMP echo response)
//
//  links:  (1)      (2)      (3)      (4)      (5)      (6)
//              ----     ----     ----     ----     ----
//  servers ---| WR |---| M1 |---| M2 |---| M3 |---| LR |--- clients
//              ----     ----     ----     ----     ----
//  ns-3 node IDs:
//  nodes 0-2    3        4        5        6         7       8-10
//
// The use of 'server' and 'client' terminology is consistent with RFC 1983
// terminology in that home clients are requesting data from Internet servers
//
// - The box WR is a WAN router, aggregating all server links
// - The box M1 is notionally an access network headend such as a CMTS or BRAS
// - The box M2 is notionally an access network CPE device such as a cable or DSL modem
// - The box M3 is notionally a home router (HR) running cake or FQ-CoDel
// - The box LR is another LAN router, aggregating all client links (to home devices)
// - Three servers are connected to WR, three clients are connected to LR
//
// clients and servers are configured for ICMP measurements and TCP throughput
// and latency measurements in the downstream direction
//
// Depending on the scenario, the middleboxes and endpoints will be
// configured differently.  Scenarios are not configured explicitly by
// scenario name but instead by combinations of input arguments.
//
// All link rates are enforced by a point-to-point (P2P) ns-3 model with full
// duplex operation.  The link rate and delays are enforced by this model
// (in contrast to netem and shaping in the testbed).  Dynamic queue limits
// (BQL) are enabled to allow for queueing to occur at the priority queue layer;
// the notional P2P hardware device queue is limited to three packets.
//
// One-way link delays and link rates
// -----------------------------------
// (1) configurable delay, 1000 Mbps
// (2) 1us delay, 1000 Mbps
// (3) 1us delay, 1000 Mbps ("control" case) or configurable rate towards M2
// (4) 1us delay, 1000 Mbps
// (5) 1us delay, 1000 Mbps towards M3, 1us delay, configurable rate towards LR
// (6) 1us delay, 1000 Mbps
//
// The setup is the 'consecutive bottleneck' scenario from Sebastian Moeller,
// corresponding to scenarios 5 and 6 of Pete Heist's experiments, although
// a single bottleneck can also be configured.
//
// Link 3 is the configured rate from scenario 5 of Pete Heist's experiments
// (defaulting 50 Mbps).  Link 5 is configured to be a fraction of link 3
// (defaulting to 95%) to set up the CAKE-like bandwidth shaping [1] conditions.  // Both link 3 rate  and link 5 fraction of link 3 rate are configurable at 
// the command line.
// 
// [1] Toke Høiland-Jørgensen, Dave Täht, and Jonathan Morton, "Piece of CAKE: 
//     A Comprehensive Queue Management Solution for Home Gateways", arXiv
//     ePrint 1804.07617, 2018.
//
// In addition, the scenario can be changed to avoid the M1 bottleneck,
// in which case the link 3 rate is not slightly above that of link 5 but
// is instead set to a higher rate to avoid a bottleneck.  This leaves M3 to
// LR (link 5) as the only bottleneck.  We call this a 'control' scenario.
//
// By default, ns-3 FQ-CoDel model is installed on all interfaces.  When
// M1 is a notional FIFO, the ns-3 Fifo model is used on M1, with a size of
// 5000 packets (to avoid tail drop in these experiments).
//
// The ns-3 FQ-CoDel model uses ns-3 defaults:
// - 100ms interval
// - 5ms target
// - drop batch size of 64 packets
// - minbytes of 1500
//
// Default simulation time is 70 sec.  For single flow experiments, the flow is
// started at simulation time 5 sec; if a second flow is used, it starts
// at 15 sec.
//
// ping frequency is set at 100ms, corresponding to Pete Heist's setup.
// Note that pings may miss the peak of queue buildups for short-lived flows;
// hence, we trace also the M1 queue length expressed in units of time at
// the bottleneck link rate.
//
// A command-line option to enable a step-threshold Immediate AQM feedback 
// from the CoDel queue model is provided.
//
// Measure:
//  - ping RTT
//  - TCP RTT estimate
//  - TCP throughput
//
// IPv4 addressing
// ----------------------------
// pingServer       10.1.1.2 (ping source)
// firstServer      10.1.2.2 (data sender)
// secondServer     10.1.3.2 (data sender)
// pingClient       192.168.1.2
// firstClient      192.168.2.2
// secondClient     192.168.3.2
//
// Program options
// ---------------
//    --firstTcpType:       First TCP type (cubic, dctcp, or reno) [cubic]
//    --secondTcpType:      Second TCP type (cubic, dctcp, or reno) [cubic]
//    --m3QueueType:        M3 queue type (fq or codel) [fq]
//    --queueUseEcn:        Whether queue uses ECN
//    --baseRtt:            base RTT [80ms]
//    --controlScenario:    control scenario (disable M1 bottleneck) [false]
//    --useIaqm:            use immediate AQM on CoDel queues [false]
//    --iaqmThreshold:      CoDel IAQM threshold [1ms]
//    --link3rate:          data rate of link 3 for FIFO scenarios [50Mbps]
//    --link5rateRatio:     ratio of data rate of link 5 to link 3 [0.95]
//    --stopTime:           simulation stop time [70s]
//    --enablePcap:         enable Pcap [false]
//    (additional arguments to control trace names)
//

#include <iostream>
#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/dce-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TsvwgScenarios");

uint32_t g_firstBytesReceived = 0;
uint32_t g_secondBytesReceived = 0;
uint32_t g_marksObserved = 0;
uint32_t g_dropsObserved = 0;

void
TraceFirstCwnd (std::ofstream* ofStream, uint32_t oldCwnd, uint32_t newCwnd)
{
  // TCP segment size is configured below to be 1448 bytes
  // so that we can report cwnd in units of segments
  *ofStream << Simulator::Now ().GetSeconds () << " " << static_cast<double> (newCwnd) / 1448 << std::endl;
}

void
TraceFirstRtt (std::ofstream* ofStream, Time oldRtt, Time newRtt)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << newRtt.GetSeconds () * 1000 << std::endl;
}

void
TraceSecondCwnd (std::ofstream* ofStream, uint32_t oldCwnd, uint32_t newCwnd)
{
  // TCP segment size is configured below to be 1448 bytes
  // so that we can report cwnd in units of segments
  *ofStream << Simulator::Now ().GetSeconds () << " " << static_cast<double> (newCwnd) / 1448 << std::endl;
}

void
TraceSecondRtt (std::ofstream* ofStream, Time oldRtt, Time newRtt)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << newRtt.GetSeconds () * 1000 << std::endl;
}

void
TracePingRtt (std::ofstream* ofStream, Time rtt)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << rtt.GetSeconds () * 1000 << std::endl;
}

void
TraceFirstRx (Ptr<const Packet> packet, const Address &address)
{
  g_firstBytesReceived += packet->GetSize ();
}

void
TraceSecondRx (Ptr<const Packet> packet, const Address &address)
{
  g_secondBytesReceived += packet->GetSize ();
}

void
TraceM1Drop (std::ofstream* ofStream, Ptr<const QueueDiscItem> item)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::hex << item->Hash () << std::endl;
  g_dropsObserved++;
}

void
TraceM3Drop (std::ofstream* ofStream, Ptr<const QueueDiscItem> item)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::hex << item->Hash () << std::endl;
}

void
TraceM3Mark (std::ofstream* ofStream, Ptr<const QueueDiscItem> item, const char* reason)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::hex << item->Hash () << std::endl;
  g_marksObserved++;
}

void
TraceM1QueueLength (std::ofstream* ofStream, DataRate m1LinkRate, uint32_t oldVal, uint32_t newVal)
{
  // output in units of ms
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::fixed << static_cast<double> (newVal * 8) / (m1LinkRate.GetBitRate () / 1000) << std::endl;
}

void
TraceM3QueueLength (std::ofstream* ofStream, DataRate m3LinkRate, uint32_t oldVal, uint32_t newVal)
{
  // output in units of ms
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::fixed << static_cast<double> (newVal * 8) / (m3LinkRate.GetBitRate () / 1000) << std::endl;
}

void
TraceDropsFrequency (std::ofstream* ofStream, Time dropsSamplingInterval)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << g_dropsObserved << std::endl;
  g_dropsObserved = 0;
  Simulator::Schedule (dropsSamplingInterval, &TraceDropsFrequency, ofStream, dropsSamplingInterval);
}

void
TraceMarksFrequency (std::ofstream* ofStream, Time marksSamplingInterval)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << g_marksObserved << std::endl;
  g_marksObserved = 0;
  Simulator::Schedule (marksSamplingInterval, &TraceMarksFrequency, ofStream, marksSamplingInterval);
}

void
TraceFirstThroughput (std::ofstream* ofStream, Time throughputInterval)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << g_firstBytesReceived * 8 / throughputInterval.GetSeconds () / 1e6 << std::endl;
  g_firstBytesReceived = 0;
  Simulator::Schedule (throughputInterval, &TraceFirstThroughput, ofStream, throughputInterval);
}

void
TraceSecondThroughput (std::ofstream* ofStream, Time throughputInterval)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << g_secondBytesReceived * 8 / throughputInterval.GetSeconds () / 1e6 << std::endl;
  g_secondBytesReceived = 0;
  Simulator::Schedule (throughputInterval, &TraceSecondThroughput, ofStream, throughputInterval);
}

void
ScheduleFirstTcpCwndTraceConnection (std::ofstream* ofStream)
{
  Config::ConnectWithoutContextFailSafe ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeBoundCallback (&TraceFirstCwnd, ofStream));
}

void
ScheduleFirstTcpRttTraceConnection (std::ofstream* ofStream)
{
  Config::ConnectWithoutContextFailSafe ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeBoundCallback (&TraceFirstRtt, ofStream));
}

void
ScheduleFirstPacketSinkConnection (void)
{
  Config::ConnectWithoutContextFailSafe ("/NodeList/9/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&TraceFirstRx));
}

void
ScheduleSecondTcpCwndTraceConnection (std::ofstream* ofStream)
{
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeBoundCallback (&TraceSecondCwnd, ofStream));
}

void
ScheduleSecondTcpRttTraceConnection (std::ofstream* ofStream)
{
  Config::ConnectWithoutContext ("/NodeList/2/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeBoundCallback (&TraceSecondRtt, ofStream));
}

void
ScheduleSecondPacketSinkConnection (void)
{
  Config::ConnectWithoutContext ("/NodeList/10/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&TraceSecondRx));
}

int
main (int argc, char *argv[])
{
  ////////////////////////////////////////////////////////////
  // variables not configured at command line               //
  ////////////////////////////////////////////////////////////
  Time stopTime = Seconds (70);
  Time baseRtt = MilliSeconds (80);
  uint32_t pingSize = 100; // bytes
  bool useIaqm = false;
  bool dceSender = false;
  bool dceReceiver = false;
  bool queueUseEcn = false;
  Time pingInterval = MilliSeconds (100);
  Time marksSamplingInterval = MilliSeconds (100);
  Time throughputSamplingInterval = MilliSeconds (200);
  DataRate link3Rate ("50Mbps");
  double link5RateRatio = 0.95;
  std::string pingTraceFile = "tsvwg-scenarios-ping.dat";
  std::string firstTcpRttTraceFile = "tsvwg-scenarios-first-tcp-rtt.dat";
  std::string firstTcpCwndTraceFile = "tsvwg-scenarios-first-tcp-cwnd.dat";
  std::string firstTcpThroughputTraceFile = "tsvwg-scenarios-first-tcp-throughput.dat";
  std::string secondTcpRttTraceFile = "tsvwg-scenarios-second-tcp-rtt.dat";
  std::string secondTcpCwndTraceFile = "tsvwg-scenarios-second-tcp-cwnd.dat";
  std::string secondTcpThroughputTraceFile = "tsvwg-scenarios-second-tcp-throughput.dat";
  std::string m1DropTraceFile = "tsvwg-scenarios-m1-drops.dat";
  std::string m1DropsFrequencyTraceFile = "tsvwg-scenarios-m1-drops-frequency.dat";
  std::string m1LengthTraceFile = "tsvwg-scenarios-m1-length.dat";
  std::string m3MarkTraceFile = "tsvwg-scenarios-m3-marks.dat";
  std::string m3MarksFrequencyTraceFile = "tsvwg-scenarios-m3-marks-frequency.dat";
  std::string m3DropTraceFile = "tsvwg-scenarios-m3-drops.dat";
  std::string m3LengthTraceFile = "tsvwg-scenarios-m3-length.dat";

  ////////////////////////////////////////////////////////////
  // variables configured at command line                   //
  ////////////////////////////////////////////////////////////
  bool enablePcap = false;
  bool controlScenario = false;
  Time iaqmThreshold = MilliSeconds (1);
  std::string firstTcpType = "cubic";
  std::string secondTcpType = "";
  bool enableSecondTcp = false;
  std::string m3QueueType = "fq";

  ////////////////////////////////////////////////////////////
  // Override ns-3 defaults                                 //
  ////////////////////////////////////////////////////////////
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1448));
  // Increase default buffer sizes to improve throughput over long delay paths
  Config::SetDefault ("ns3::TcpSocket::SndBufSize",UintegerValue (8192000));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize",UintegerValue (8192000));
  Config::SetDefault ("ns3::TcpSocket::InitialCwnd", UintegerValue (10));
  Config::SetDefault ("ns3::TcpL4Protocol::RecoveryType", TypeIdValue (TcpPrrRecovery::GetTypeId ()));
  // Avoid tail drops in the M1 queue for high bandwidth scenarios
  Config::SetDefault ("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize ("5000p")));

  ////////////////////////////////////////////////////////////
  // command-line argument parsing                          //
  ////////////////////////////////////////////////////////////
  CommandLine cmd;
  cmd.AddValue ("firstTcpType", "First TCP type (cubic, dctcp, or reno)", firstTcpType);
  cmd.AddValue ("secondTcpType", "Second TCP type (cubic, dctcp, or reno)", secondTcpType);
  cmd.AddValue ("m3QueueType", "M3 queue type (fq or codel)", m3QueueType);
  cmd.AddValue ("baseRtt", "base RTT", baseRtt);
  cmd.AddValue ("controlScenario", "control scenario (disable M1 bottleneck)", controlScenario);
  cmd.AddValue ("useIaqm", "use immediate AQM on CoDel queues", useIaqm);
  cmd.AddValue ("iaqmThreshold", "CoDel IAQM threshold", iaqmThreshold);
  cmd.AddValue ("link3rate", "data rate of link 3 for FIFO scenarios", link3Rate);
  cmd.AddValue ("link5rateRatio", "ratio of data rate of link 5 to link 3", link5RateRatio);
  cmd.AddValue ("stopTime", "simulation stop time", stopTime);
  cmd.AddValue ("dceSender", "DCE sender", dceSender);
  cmd.AddValue ("dceReceiver", "DCE receiver", dceReceiver);
  cmd.AddValue ("queueUseEcn", "use ECN on queue", queueUseEcn);
  cmd.AddValue ("enablePcap", "enable Pcap", enablePcap);
  cmd.AddValue ("pingTraceFile", "filename for ping tracing", pingTraceFile);
  cmd.AddValue ("firstTcpRttTraceFile", "filename for rtt tracing", firstTcpRttTraceFile);
  cmd.AddValue ("firstTcpCwndTraceFile", "filename for cwnd tracing", firstTcpCwndTraceFile);
  cmd.AddValue ("firstTcpThroughputTraceFile", "filename for throughput tracing", firstTcpThroughputTraceFile);
  cmd.AddValue ("secondTcpRttTraceFile", "filename for second rtt tracing", secondTcpRttTraceFile);
  cmd.AddValue ("secondTcpCwndTraceFile", "filename for second cwnd tracing", secondTcpCwndTraceFile);
  cmd.AddValue ("secondTcpThroughputTraceFile", "filename for second throughput tracing", secondTcpThroughputTraceFile);
  cmd.AddValue ("m1DropTraceFile", "filename for m1 drops tracing", m1DropTraceFile);
  cmd.AddValue ("m1DropsFrequencyTraceFile", "filename for m1 drop frequency tracing", m1DropsFrequencyTraceFile);
  cmd.AddValue ("m1LengthTraceFile", "filename for m1 queue length tracing", m1LengthTraceFile);
  cmd.AddValue ("m3MarkTraceFile", "filename for m3 mark tracing", m3MarkTraceFile);
  cmd.AddValue ("m3MarksFrequencyTraceFile", "filename for m3 mark frequency tracing", m3MarksFrequencyTraceFile);
  cmd.AddValue ("m3DropTraceFile", "filename for m3 drop tracing", m3DropTraceFile);
  cmd.AddValue ("m3LengthTraceFile", "filename for m3 queue length tracing", m3LengthTraceFile);
  cmd.Parse (argc, argv);

  if (dceSender || dceReceiver)
    {
      GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
    }

  Time oneWayDelay = baseRtt/2;

  TypeId firstTcpTypeId;
  if (firstTcpType == "reno")
    {
      firstTcpTypeId = TcpLinuxReno::GetTypeId ();
    }
  else if (firstTcpType == "cubic")
    {
      firstTcpTypeId = TcpCubic::GetTypeId ();
    }
  else
    {
      NS_FATAL_ERROR ("Fatal error:  tcp unsupported");
    }
  TypeId secondTcpTypeId;
  if (secondTcpType == "reno")
    {
      enableSecondTcp = true;
      secondTcpTypeId = TcpLinuxReno::GetTypeId ();
    }
  else if (secondTcpType == "cubic")
    {
      enableSecondTcp = true;
      secondTcpTypeId = TcpCubic::GetTypeId ();
    }
  else if (secondTcpType == "")
    {
      NS_LOG_DEBUG ("No second TCP selected");
    }
  else
    {
      NS_FATAL_ERROR ("Fatal error:  tcp unsupported");
    }
  TypeId m3QueueTypeId;
  if (m3QueueType == "fq")
    {
      m3QueueTypeId = FqCoDelQueueDisc::GetTypeId ();
    }
  else if (m3QueueType == "codel")
    {
      m3QueueTypeId = CoDelQueueDisc::GetTypeId ();
    }
  else
    {
      NS_FATAL_ERROR ("Fatal error:  m3QueueType unsupported");
    }

  if (queueUseEcn)
    {
      Config::SetDefault ("ns3::CoDelQueueDisc::UseEcn", BooleanValue (true));
    }
  Config::SetDefault ("ns3::TcpSocketBase::UseEcn", StringValue ("On"));
  if (useIaqm)
    {
      Config::SetDefault ("ns3::CoDelQueueDisc::UseIaqm", BooleanValue (true));
      Config::SetDefault ("ns3::CoDelQueueDisc::IaqmThreshold", TimeValue (iaqmThreshold));
    }

  // Report on configuration
  NS_LOG_DEBUG ("first TCP: " << firstTcpTypeId.GetName () << "; second TCP: " << secondTcpTypeId.GetName () << "; M3 queue: " << m3QueueTypeId.GetName () << "; control: " << controlScenario << "; iaqm: " << useIaqm << "; iaqmThreshold: " << iaqmThreshold.GetSeconds () * 1000 << "ms");

  std::ofstream pingOfStream;
  pingOfStream.open (pingTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpRttOfStream;
  firstTcpRttOfStream.open (firstTcpRttTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpCwndOfStream;
  firstTcpCwndOfStream.open (firstTcpCwndTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpThroughputOfStream;
  firstTcpThroughputOfStream.open (firstTcpThroughputTraceFile.c_str (), std::ofstream::out);
  std::ofstream secondTcpRttOfStream;
  secondTcpRttOfStream.open (secondTcpRttTraceFile.c_str (), std::ofstream::out);
  std::ofstream secondTcpCwndOfStream;
  secondTcpCwndOfStream.open (secondTcpCwndTraceFile.c_str (), std::ofstream::out);
  std::ofstream secondTcpThroughputOfStream;
  secondTcpThroughputOfStream.open (secondTcpThroughputTraceFile.c_str (), std::ofstream::out);
  std::ofstream m1DropOfStream;
  m1DropOfStream.open (m1DropTraceFile.c_str (), std::ofstream::out);
  std::ofstream m3DropOfStream;
  m3DropOfStream.open (m3DropTraceFile.c_str (), std::ofstream::out);
  std::ofstream m3MarkOfStream;
  m3MarkOfStream.open (m3MarkTraceFile.c_str (), std::ofstream::out);
  std::ofstream m1DropsFrequencyOfStream;
  m1DropsFrequencyOfStream.open (m1DropsFrequencyTraceFile.c_str (), std::ofstream::out);
  std::ofstream m3MarksFrequencyOfStream;
  m3MarksFrequencyOfStream.open (m3MarksFrequencyTraceFile.c_str (), std::ofstream::out);
  std::ofstream m1LengthOfStream;
  m1LengthOfStream.open (m1LengthTraceFile.c_str (), std::ofstream::out);
  std::ofstream m3LengthOfStream;
  m3LengthOfStream.open (m3LengthTraceFile.c_str (), std::ofstream::out);

  ////////////////////////////////////////////////////////////
  // scenario setup                                         //
  ////////////////////////////////////////////////////////////
  Ptr<Node> pingServer = CreateObject<Node> ();
  Ptr<Node> firstServer = CreateObject<Node> ();
  Ptr<Node> secondServer = CreateObject<Node> ();
  Ptr<Node> wanRouter = CreateObject<Node> ();
  Ptr<Node> M1 = CreateObject<Node> ();
  Ptr<Node> M2 = CreateObject<Node> ();
  Ptr<Node> M3 = CreateObject<Node> ();
  Ptr<Node> lanRouter = CreateObject<Node> ();
  Ptr<Node> pingClient = CreateObject<Node> ();
  Ptr<Node> firstClient = CreateObject<Node> ();
  Ptr<Node> secondClient = CreateObject<Node> ();

  // Device containers
  NetDeviceContainer pingServerDevices;
  NetDeviceContainer firstServerDevices;
  NetDeviceContainer secondServerDevices;
  NetDeviceContainer wanRouterM1Devices;
  NetDeviceContainer M1M2Devices;
  NetDeviceContainer M2M3Devices;
  NetDeviceContainer M3LanRouterDevices;
  NetDeviceContainer pingClientDevices;
  NetDeviceContainer firstClientDevices;
  NetDeviceContainer secondClientDevices;

  PointToPointHelper p2p;
  if (dceSender)
    {
      // Set large queue size on DCE links because BQL is not in effect
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("1000p")));
    }
  else
    {
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("3p")));
    }
  p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1000Mbps")));
  // Add delay only on the WAN links
  p2p.SetChannelAttribute ("Delay", TimeValue (oneWayDelay));
  pingServerDevices = p2p.Install (wanRouter, pingServer);
  firstServerDevices = p2p.Install (wanRouter, firstServer);
  secondServerDevices = p2p.Install (wanRouter, secondServer);
  p2p.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  wanRouterM1Devices = p2p.Install (wanRouter, M1);
  M1M2Devices = p2p.Install (M1, M2);
  M2M3Devices = p2p.Install (M2, M3);
  M3LanRouterDevices = p2p.Install (M3, lanRouter);
  if (dceReceiver)
    {
      // Set large queue size on DCE links because BQL is not in effect
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("1000p")));
    }
  else
    {
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("3p")));
    }
  pingClientDevices = p2p.Install (lanRouter, pingClient);
  firstClientDevices = p2p.Install (lanRouter, firstClient);
  secondClientDevices = p2p.Install (lanRouter, secondClient);

  // Limit the bandwidth on the M3->lanRouter interface (link 5)
  // Note:  in the "control" cases, link5 rate is still set based on the
  // configured "link3Rate" value, even though link 3 is left at 1 Gbps
  Ptr<PointToPointNetDevice> p = M3LanRouterDevices.Get (0)->GetObject<PointToPointNetDevice> ();
  DataRate link5Rate (link5RateRatio * link3Rate.GetBitRate ());
  p->SetAttribute ("DataRate", DataRateValue (link5Rate));

  // If not a "control" scenario, limit link 3 accordingly
  if (!controlScenario)
    {
      p = M1M2Devices.Get (0)->GetObject<PointToPointNetDevice> ();
      p->SetAttribute ("DataRate", DataRateValue (link3Rate));
    }

  DceManagerHelper dceManager;
  // Configure the Linux stack on the nodes
  dceManager.SetTaskManagerAttribute ("FiberManagerType",
                                      StringValue ("UcontextFiberManager"));

  dceManager.SetNetworkStack ("ns3::LinuxSocketFdFactory",
                              "Library", StringValue ("liblinux.so"));

  LinuxStackHelper stackLinux;

  InternetStackHelper stackHelper;
  stackHelper.Install (pingServer);
  Ptr<TcpL4Protocol> proto;
  if (dceSender)
    {
      stackLinux.Install (firstServer);
      dceManager.Install (firstServer);
      stackLinux.SysctlSet (firstServer, ".net.ipv4.tcp_congestion_control", firstTcpType);
      stackLinux.SysctlSet (firstServer, ".net.ipv4.tcp_ecn", "1");
    }
  else
    {
      stackHelper.Install (firstServer);
      proto = firstServer->GetObject<TcpL4Protocol> ();
      proto->SetAttribute ("SocketType", TypeIdValue (firstTcpTypeId));
    }
  stackHelper.Install (secondServer);
  stackHelper.Install (wanRouter);
  stackHelper.Install (M1);
  stackHelper.Install (M2);
  stackHelper.Install (M3);
  stackHelper.Install (lanRouter);
  stackHelper.Install (pingClient);

  if (dceReceiver)
    {
      stackLinux.Install (firstClient);
      dceManager.Install (firstClient);
      stackLinux.SysctlSet (firstClient, ".net.ipv4.tcp_congestion_control", secondTcpType);
      stackLinux.SysctlSet (firstClient, ".net.ipv4.tcp_ecn", "1");
    }
  else
    {
      stackHelper.Install (firstClient);
      // Set the per-node TCP type here
      proto = firstClient->GetObject<TcpL4Protocol> ();
      proto->SetAttribute ("SocketType", TypeIdValue (firstTcpTypeId));
    }
  stackHelper.Install (secondClient);

  if (enableSecondTcp)
    {
      proto = secondClient->GetObject<TcpL4Protocol> ();
      proto->SetAttribute ("SocketType", TypeIdValue (secondTcpTypeId));
      proto = secondServer->GetObject<TcpL4Protocol> ();
      proto->SetAttribute ("SocketType", TypeIdValue (secondTcpTypeId));
    }

  // InternetStackHelper will install a base TrafficControLayer on the node,
  // but the Ipv4AddressHelper below will install the default FqCoDelQueueDisc
  // on all single device nodes.  The below code overrides the configuration
  // that is normally done by the Ipv4AddressHelper::Install() method by
  // instead explicitly configuring the queue discs we want on each device.
  TrafficControlHelper tchFq;
  tchFq.SetRootQueueDisc ("ns3::FqCoDelQueueDisc");
  tchFq.SetQueueLimits ("ns3::DynamicQueueLimits", "HoldTime", StringValue ("1ms"));
  tchFq.Install (pingServerDevices);
  if (dceSender)
    {
      tchFq.Install (firstServerDevices.Get (0));
    }
  else
    {
      tchFq.Install (firstServerDevices);
    }
  tchFq.Install (secondServerDevices);
  tchFq.Install (wanRouterM1Devices);
  tchFq.Install (M1M2Devices.Get (1));  // M2 queue for link 3
  tchFq.Install (M2M3Devices);
  tchFq.Install (M3LanRouterDevices.Get (1));  //  M3 queue for link 5
  tchFq.Install (pingClientDevices);
  if (dceReceiver)
    {
      tchFq.Install (firstClientDevices.Get (0));
    }
  else
    {
      tchFq.Install (firstClientDevices);
    }
  tchFq.Install (secondClientDevices);
  // Install FIFO on M1 queue for link 3
  TrafficControlHelper tchM1;
  tchM1.SetRootQueueDisc ("ns3::FifoQueueDisc");
  tchM1.SetQueueLimits ("ns3::DynamicQueueLimits", "HoldTime", StringValue ("1ms"));
  tchM1.Install (M1M2Devices.Get (0));
  // Install queue for M3 queue for link 3
  TrafficControlHelper tchM3;
  tchM3.SetRootQueueDisc (m3QueueTypeId.GetName ());
  tchM3.SetQueueLimits ("ns3::DynamicQueueLimits", "HoldTime", StringValue ("1ms"));
  tchM3.Install (M3LanRouterDevices.Get (0));

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer pingServerIfaces = ipv4.Assign (pingServerDevices);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer firstServerIfaces = ipv4.Assign (firstServerDevices);
  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer secondServerIfaces = ipv4.Assign (secondServerDevices);
  ipv4.SetBase ("172.16.1.0", "255.255.255.0");
  Ipv4InterfaceContainer wanRouterM1Ifaces = ipv4.Assign (wanRouterM1Devices);
  ipv4.SetBase ("172.16.2.0", "255.255.255.0");
  Ipv4InterfaceContainer M1M2Ifaces = ipv4.Assign (M1M2Devices);
  ipv4.SetBase ("172.16.3.0", "255.255.255.0");
  Ipv4InterfaceContainer M2M3Ifaces = ipv4.Assign (M2M3Devices);
  ipv4.SetBase ("172.16.4.0", "255.255.255.0");
  Ipv4InterfaceContainer M3LanRouterIfaces = ipv4.Assign (M3LanRouterDevices);
  ipv4.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer pingClientIfaces = ipv4.Assign (pingClientDevices);
  ipv4.SetBase ("192.168.2.0", "255.255.255.0");
  Ipv4InterfaceContainer firstClientIfaces = ipv4.Assign (firstClientDevices);
  ipv4.SetBase ("192.168.3.0", "255.255.255.0");
  Ipv4InterfaceContainer secondClientIfaces = ipv4.Assign (secondClientDevices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  if (dceSender)
    {
      // default routing in the DCE nodes
      Ipv4Address firstServerToWRAddr = firstServerIfaces.GetAddress (0, 0); 
      std::ostringstream streamFirstServer;
      streamFirstServer << "route add default via ";
      streamFirstServer << firstServerToWRAddr;
      streamFirstServer << " dev sim0";
      LinuxStackHelper::RunIp (firstServer, Seconds (0.1), streamFirstServer.str ());
    }
  if (dceReceiver)
    {
      Ipv4Address firstClientToLRAddr = firstClientIfaces.GetAddress (0, 0); 
      std::ostringstream streamFirstClient;
      streamFirstClient << "route add default via ";
      streamFirstClient << firstClientToLRAddr;
      streamFirstClient << " dev sim0";
      LinuxStackHelper::RunIp (firstClient, Seconds (0.1), streamFirstClient.str ());
    }
  ////////////////////////////////////////////////////////////
  // application setup                                      //
  ////////////////////////////////////////////////////////////
  V4PingHelper pingHelper ("192.168.1.2");
  pingHelper.SetAttribute ("Interval", TimeValue (pingInterval));
  pingHelper.SetAttribute ("Size", UintegerValue (pingSize));
  ApplicationContainer pingContainer = pingHelper.Install (pingServer);
  Ptr<V4Ping> v4Ping = pingContainer.Get (0)->GetObject<V4Ping> ();
  v4Ping->TraceConnectWithoutContext ("Rtt", MakeBoundCallback (&TracePingRtt, &pingOfStream));
  pingContainer.Start (Seconds (1));
  pingContainer.Stop (stopTime - Seconds (1));

  ApplicationContainer firstApp;
  uint16_t firstPort = 5000;
  if (dceSender)
    {
      BulkSendHelper tcp ("ns3::LinuxTcpSocketFactory", Address ());
      // set to large value:  e.g. 1000 Mb/s for 60 seconds = 7500000000 bytes
      tcp.SetAttribute ("MaxBytes", UintegerValue (7500000000));
      // Configure first TCP client/server pair
      InetSocketAddress firstDestAddress (firstClientIfaces.GetAddress (1), firstPort);
      tcp.SetAttribute ("Remote", AddressValue (firstDestAddress));
      firstApp = tcp.Install (firstServer);
      firstApp.Start (Seconds (5));
      firstApp.Stop (stopTime - Seconds (1));
    }
  else
    {
      BulkSendHelper tcp ("ns3::TcpSocketFactory", Address ());
      // set to large value:  e.g. 1000 Mb/s for 60 seconds = 7500000000 bytes
      tcp.SetAttribute ("MaxBytes", UintegerValue (7500000000));
      // Configure first TCP client/server pair
      InetSocketAddress firstDestAddress (firstClientIfaces.GetAddress (1), firstPort);
      tcp.SetAttribute ("Remote", AddressValue (firstDestAddress));
      firstApp = tcp.Install (firstServer);
      firstApp.Start (Seconds (5));
      firstApp.Stop (stopTime - Seconds (1));
    }

  Address firstSinkAddress (InetSocketAddress (Ipv4Address::GetAny (), firstPort));
  ApplicationContainer firstSinkApp;
  if (dceReceiver)
    {
      PacketSinkHelper firstSinkHelper ("ns3::LinuxTcpSocketFactory", firstSinkAddress);
      firstSinkApp = firstSinkHelper.Install (firstClient);
      firstSinkApp.Start (Seconds (5));
      firstSinkApp.Stop (stopTime - MilliSeconds (500));
    }
  else
    {
      PacketSinkHelper firstSinkHelper ("ns3::TcpSocketFactory", firstSinkAddress);
      firstSinkApp = firstSinkHelper.Install (firstClient);
      firstSinkApp.Start (Seconds (5));
      firstSinkApp.Stop (stopTime - MilliSeconds (500));
    }

  // Configure second TCP client/server pair
  if (enableSecondTcp)
    {
      BulkSendHelper tcp ("ns3::TcpSocketFactory", Address ());
      uint16_t secondPort = 5000;
      ApplicationContainer secondApp;
      InetSocketAddress secondDestAddress (secondClientIfaces.GetAddress (1), secondPort);
      tcp.SetAttribute ("Remote", AddressValue (secondDestAddress));
      secondApp = tcp.Install (secondServer);
      secondApp.Start (Seconds (15));
      secondApp.Stop (stopTime - Seconds (1));

      Address secondSinkAddress (InetSocketAddress (Ipv4Address::GetAny (), secondPort));
      PacketSinkHelper secondSinkHelper ("ns3::TcpSocketFactory", secondSinkAddress);
      ApplicationContainer secondSinkApp;
      secondSinkApp = secondSinkHelper.Install (secondClient);
      secondSinkApp.Start (Seconds (15));
      secondSinkApp.Stop (stopTime - MilliSeconds (500));
    }

  // Setup traces that can be hooked now
  Ptr<TrafficControlLayer> tc;
  Ptr<QueueDisc> qd;
  // Trace drops for M1
  tc = M1M2Devices.Get (0)->GetNode ()->GetObject<TrafficControlLayer> ();
  qd = tc->GetRootQueueDiscOnDevice (M1M2Devices.Get (0));
  qd->TraceConnectWithoutContext ("Drop", MakeBoundCallback (&TraceM1Drop, &m1DropOfStream));
  qd->TraceConnectWithoutContext ("BytesInQueue", MakeBoundCallback (&TraceM1QueueLength, &m1LengthOfStream, link3Rate));
  // Trace drops and marks for M3
  tc = M3LanRouterDevices.Get (0)->GetNode ()->GetObject<TrafficControlLayer> ();
  qd = tc->GetRootQueueDiscOnDevice (M3LanRouterDevices.Get (0));
  qd->TraceConnectWithoutContext ("Mark", MakeBoundCallback (&TraceM3Mark, &m3MarkOfStream));
  qd->TraceConnectWithoutContext ("Drop", MakeBoundCallback (&TraceM3Drop, &m3DropOfStream));
  qd->TraceConnectWithoutContext ("BytesInQueue", MakeBoundCallback (&TraceM3QueueLength, &m3LengthOfStream, link5Rate));

  // Setup scheduled traces; TCP traces must be hooked after socket creation
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstTcpRttTraceConnection, &firstTcpRttOfStream);
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstTcpCwndTraceConnection, &firstTcpCwndOfStream);
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstPacketSinkConnection);
  Simulator::Schedule (throughputSamplingInterval, &TraceFirstThroughput, &firstTcpThroughputOfStream, throughputSamplingInterval);
#if 0
  // Setup scheduled traces; TCP traces must be hooked after socket creation
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondTcpRttTraceConnection, &secondTcpRttOfStream);
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondTcpCwndTraceConnection, &secondTcpCwndOfStream);
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondPacketSinkConnection);
#endif
  Simulator::Schedule (throughputSamplingInterval, &TraceSecondThroughput, &secondTcpThroughputOfStream, throughputSamplingInterval);
  Simulator::Schedule (marksSamplingInterval, &TraceMarksFrequency, &m3MarksFrequencyOfStream, marksSamplingInterval);
  Simulator::Schedule (marksSamplingInterval, &TraceDropsFrequency, &m1DropsFrequencyOfStream, marksSamplingInterval);

  if (enablePcap)
    {
      p2p.EnablePcapAll ("tsvwg-scenarios", false);
    }

  Simulator::Stop (stopTime);
  Simulator::Run ();

  pingOfStream.close ();
  firstTcpCwndOfStream.close ();
  firstTcpRttOfStream.close ();
  firstTcpThroughputOfStream.close ();
  secondTcpCwndOfStream.close ();
  secondTcpRttOfStream.close ();
  secondTcpThroughputOfStream.close ();
  m1DropOfStream.close ();
  m3DropOfStream.close ();
  m3MarkOfStream.close ();
  m1DropsFrequencyOfStream.close ();
  m3MarksFrequencyOfStream.close ();
  m1LengthOfStream.close ();
  m3LengthOfStream.close ();
}

