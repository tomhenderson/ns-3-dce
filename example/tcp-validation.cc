/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Cable Television Laboratories, Inc.
 * Copyright (c) 2020 Tom Henderson (adapted for DCTCP testing)
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

// ---> downstream (primary data transfer from servers to clients)
// <--- upstream (return acks and ICMP echo response)
//
//              ----   bottleneck link    ---- 
//  servers ---| WR |--------------------| LR |--- clients
//              ----                      ----
//  ns-3 node IDs:
//  nodes 0-2    3                         4        5-7 
//
// - The box WR is notionally a WAN router, aggregating all server links
// - The box LR is notionally a LAN router, aggregating all client links
// - Three servers are connected to WR, three clients are connected to LR
//
// clients and servers are configured for ICMP measurements and TCP throughput
// and latency measurements in the downstream direction
//
// All link rates are enforced by a point-to-point (P2P) ns-3 model with full
// duplex operation.  Dynamic queue limits
// (BQL) are enabled to allow for queueing to occur at the priority queue layer;
// the notional P2P hardware device queue is limited to three packets.
//
// One-way link delays and link rates
// -----------------------------------
// (1) server to WR links, 1000 Mbps, 1us delay
// (2) bottleneck link:  configurable rate, configurable delay
// (3) client to LR links, 1000 Mbps, 1us delay
//
// By default, ns-3 FQ-CoDel model is installed on all interfaces.
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
// ping frequency is set at 100ms.
//
// A command-line option to enable a step-threshold CE threshold
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
//    --queueType:        M3 queue type (fq or codel or pie or fifo) [fq]
//    --queueUseEcn:        Whether queue uses ECN
//    --baseRtt:            base RTT [80ms]
//    --ceThreshold:        CoDel CE threshold [1ms]
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

NS_LOG_COMPONENT_DEFINE ("TcpValidation");

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
TraceFirstDctcp (std::ofstream* ofStream, uint32_t bytesMarked, uint32_t bytesAcked, double alpha)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << alpha << std::endl;
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
TraceQueueMark (std::ofstream* ofStream, Ptr<const QueueDiscItem> item, const char* reason)
{
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::hex << item->Hash () << std::endl;
  g_marksObserved++;
}

void
TraceQueueLength (std::ofstream* ofStream, DataRate queueLinkRate, uint32_t oldVal, uint32_t newVal)
{
  // output in units of ms
  *ofStream << Simulator::Now ().GetSeconds () << " " << std::fixed << static_cast<double> (newVal * 8) / (queueLinkRate.GetBitRate () / 1000) << std::endl;
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
ScheduleFirstDctcpTraceConnection (std::ofstream* ofStream)
{
  Config::ConnectWithoutContextFailSafe ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpDctcp/CongestionEstimate", MakeBoundCallback (&TraceFirstDctcp, ofStream));
}

void
ScheduleFirstPacketSinkConnection (void)
{
  Config::ConnectWithoutContextFailSafe ("/NodeList/6/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&TraceFirstRx));
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
  Config::ConnectWithoutContext ("/NodeList/7/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&TraceSecondRx));
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
  bool dceSender = false;
  bool dceReceiver = false;
  bool queueUseEcn = false;
  Time pingInterval = MilliSeconds (100);
  Time marksSamplingInterval = MilliSeconds (100);
  Time throughputSamplingInterval = MilliSeconds (200);
  DataRate linkRate ("50Mbps");
  std::string pingTraceFile = "tcp-validation-ping.dat";
  std::string firstTcpRttTraceFile = "tcp-validation-first-tcp-rtt.dat";
  std::string firstTcpCwndTraceFile = "tcp-validation-first-tcp-cwnd.dat";
  std::string firstDctcpTraceFile = "tcp-validation-dctcp-alpha.dat";
  std::string firstTcpThroughputTraceFile = "tcp-validation-first-tcp-throughput.dat";
  std::string secondTcpRttTraceFile = "tcp-validation-second-tcp-rtt.dat";
  std::string secondTcpCwndTraceFile = "tcp-validation-second-tcp-cwnd.dat";
  std::string secondTcpThroughputTraceFile = "tcp-validation-second-tcp-throughput.dat";
  std::string queueMarkTraceFile = "tcp-validation-queue-mark.dat";
  std::string queueMarksFrequencyTraceFile = "tcp-validation-queue-marks-frequency.dat";
  std::string queueLengthTraceFile = "tcp-validation-queue-length.dat";

  ////////////////////////////////////////////////////////////
  // variables configured at command line                   //
  ////////////////////////////////////////////////////////////
  bool enablePcap = false;
  Time ceThreshold = MilliSeconds (1);
  std::string firstTcpType = "cubic";
  std::string secondTcpType = "";
  bool enableSecondTcp = false;
  std::string queueType = "codel";

  ////////////////////////////////////////////////////////////
  // Override ns-3 defaults                                 //
  ////////////////////////////////////////////////////////////
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1448));
  // Increase default buffer sizes to improve throughput over long delay paths
  //Config::SetDefault ("ns3::TcpSocket::SndBufSize",UintegerValue (8192000));
  //Config::SetDefault ("ns3::TcpSocket::RcvBufSize",UintegerValue (8192000));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize",UintegerValue (32768000));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize",UintegerValue (32768000));
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
  cmd.AddValue ("queueType", "M3 queue type (fq or codel or pie)", queueType);
  cmd.AddValue ("baseRtt", "base RTT", baseRtt);
  cmd.AddValue ("ceThreshold", "CoDel CE threshold", ceThreshold);
  cmd.AddValue ("linkRate", "data rate of bottleneck link", linkRate);
  cmd.AddValue ("stopTime", "simulation stop time", stopTime);
  cmd.AddValue ("dceSender", "DCE sender", dceSender);
  cmd.AddValue ("dceReceiver", "DCE receiver", dceReceiver);
  cmd.AddValue ("queueUseEcn", "use ECN on queue", queueUseEcn);
  cmd.AddValue ("enablePcap", "enable Pcap", enablePcap);
  cmd.AddValue ("pingTraceFile", "filename for ping tracing", pingTraceFile);
  cmd.AddValue ("firstTcpRttTraceFile", "filename for rtt tracing", firstTcpRttTraceFile);
  cmd.AddValue ("firstTcpCwndTraceFile", "filename for cwnd tracing", firstTcpCwndTraceFile);
  cmd.AddValue ("firstDctcpTraceFile", "filename for DCTCP tracing", firstDctcpTraceFile);
  cmd.AddValue ("firstTcpThroughputTraceFile", "filename for throughput tracing", firstTcpThroughputTraceFile);
  cmd.AddValue ("secondTcpRttTraceFile", "filename for second rtt tracing", secondTcpRttTraceFile);
  cmd.AddValue ("secondTcpCwndTraceFile", "filename for second cwnd tracing", secondTcpCwndTraceFile);
  cmd.AddValue ("secondTcpThroughputTraceFile", "filename for second throughput tracing", secondTcpThroughputTraceFile);
  cmd.AddValue ("queueMarksFrequencyTraceFile", "filename for queue mark frequency tracing", queueMarksFrequencyTraceFile);
  cmd.AddValue ("queueLengthTraceFile", "filename for queue queue length tracing", queueLengthTraceFile);
  cmd.Parse (argc, argv);

  //LogComponentEnable ("TcpSocketBase", (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_NODE | LOG_PREFIX_TIME | LOG_LEVEL_ALL));
  //LogComponentEnable ("TcpCubic", (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_NODE | LOG_PREFIX_TIME | LOG_LEVEL_ALL));
  //LogComponentEnable ("TcpDctcp", (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_NODE | LOG_PREFIX_TIME | LOG_LEVEL_ALL));
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
  else if (firstTcpType == "dctcp")
    {
      firstTcpTypeId = TcpDctcp::GetTypeId ();
      Config::SetDefault ("ns3::CoDelQueueDisc::CeThreshold", TimeValue (ceThreshold));
      Config::SetDefault ("ns3::FqCoDelQueueDisc::CeThreshold", TimeValue (ceThreshold));
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
  else if (secondTcpType == "dctcp")
    {
      secondTcpTypeId = TcpDctcp::GetTypeId ();
    }
  else if (secondTcpType == "")
    {
      NS_LOG_DEBUG ("No second TCP selected");
    }
  else
    {
      NS_FATAL_ERROR ("Fatal error:  tcp unsupported");
    }
  TypeId queueTypeId;
  if (queueType == "fq")
    {
      queueTypeId = FqCoDelQueueDisc::GetTypeId ();
    }
  else if (queueType == "codel")
    {
      queueTypeId = CoDelQueueDisc::GetTypeId ();
    }
  else if (queueType == "pie")
    {
      queueTypeId = PieQueueDisc::GetTypeId ();
    }
  else if (queueType == "red")
    {
      queueTypeId = RedQueueDisc::GetTypeId ();
    }
  else
    {
      NS_FATAL_ERROR ("Fatal error:  queueType unsupported");
    }

  if (queueUseEcn)
    {
      Config::SetDefault ("ns3::CoDelQueueDisc::UseEcn", BooleanValue (true));
      Config::SetDefault ("ns3::FqCoDelQueueDisc::UseEcn", BooleanValue (true));
      Config::SetDefault ("ns3::PieQueueDisc::UseEcn", BooleanValue (true));
    }
  Config::SetDefault ("ns3::TcpSocketBase::UseEcn", StringValue ("On"));

  // Report on configuration
  NS_LOG_DEBUG ("first TCP: " << firstTcpTypeId.GetName () << "; second TCP: " << secondTcpTypeId.GetName () << "; queue: " << queueTypeId.GetName () << "; ceThreshold: " << ceThreshold.GetSeconds () * 1000 << "ms");

  std::ofstream pingOfStream;
  pingOfStream.open (pingTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpRttOfStream;
  firstTcpRttOfStream.open (firstTcpRttTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpCwndOfStream;
  firstTcpCwndOfStream.open (firstTcpCwndTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpThroughputOfStream;
  firstTcpThroughputOfStream.open (firstTcpThroughputTraceFile.c_str (), std::ofstream::out);
  std::ofstream firstTcpDctcpOfStream;
  if (firstTcpType == "dctcp")
    {
      firstTcpDctcpOfStream.open (firstDctcpTraceFile.c_str (), std::ofstream::out);
    }
  std::ofstream secondTcpRttOfStream;
  secondTcpRttOfStream.open (secondTcpRttTraceFile.c_str (), std::ofstream::out);
  std::ofstream secondTcpCwndOfStream;
  secondTcpCwndOfStream.open (secondTcpCwndTraceFile.c_str (), std::ofstream::out);
  std::ofstream secondTcpThroughputOfStream;
  secondTcpThroughputOfStream.open (secondTcpThroughputTraceFile.c_str (), std::ofstream::out);
  std::ofstream queueMarkOfStream;
  queueMarkOfStream.open (queueMarkTraceFile.c_str (), std::ofstream::out);
  std::ofstream queueMarksFrequencyOfStream;
  queueMarksFrequencyOfStream.open (queueMarksFrequencyTraceFile.c_str (), std::ofstream::out);
  std::ofstream queueLengthOfStream;
  queueLengthOfStream.open (queueLengthTraceFile.c_str (), std::ofstream::out);

  ////////////////////////////////////////////////////////////
  // scenario setup                                         //
  ////////////////////////////////////////////////////////////
  Ptr<Node> pingServer = CreateObject<Node> ();
  Ptr<Node> firstServer = CreateObject<Node> ();
  Ptr<Node> secondServer = CreateObject<Node> ();
  Ptr<Node> wanRouter = CreateObject<Node> ();
  Ptr<Node> lanRouter = CreateObject<Node> ();
  Ptr<Node> pingClient = CreateObject<Node> ();
  Ptr<Node> firstClient = CreateObject<Node> ();
  Ptr<Node> secondClient = CreateObject<Node> ();

  // Device containers
  NetDeviceContainer pingServerDevices;
  NetDeviceContainer firstServerDevices;
  NetDeviceContainer secondServerDevices;
  NetDeviceContainer wanLanDevices;
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
  p2p.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  pingServerDevices = p2p.Install (wanRouter, pingServer);
  firstServerDevices = p2p.Install (wanRouter, firstServer);
  secondServerDevices = p2p.Install (wanRouter, secondServer);
  p2p.SetChannelAttribute ("Delay", TimeValue (oneWayDelay));
  wanLanDevices = p2p.Install (wanRouter, lanRouter);
  if (dceReceiver)
    {
      // Set large queue size on DCE links because BQL is not in effect
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("1000p")));
    }
  else
    {
      p2p.SetQueue ("ns3::DropTailQueue", "MaxSize", QueueSizeValue (QueueSize ("3p")));
    }
  p2p.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  pingClientDevices = p2p.Install (lanRouter, pingClient);
  firstClientDevices = p2p.Install (lanRouter, firstClient);
  secondClientDevices = p2p.Install (lanRouter, secondClient);

  // Limit the bandwidth on the wanRouter->lanRouter interface
  Ptr<PointToPointNetDevice> p = wanLanDevices.Get (0)->GetObject<PointToPointNetDevice> ();
  p->SetAttribute ("DataRate", DataRateValue (linkRate));

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
  stackHelper.Install (lanRouter);
  stackHelper.Install (pingClient);

  if (dceReceiver)
    {
      stackLinux.Install (firstClient);
      dceManager.Install (firstClient);
      stackLinux.SysctlSet (firstClient, ".net.ipv4.tcp_congestion_control", firstTcpType);
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
      // omit device (1) since it is DCE
      tchFq.Install (firstServerDevices.Get (0));
      tchFq.Install (secondServerDevices.Get (0));
    }
  else
    {
      tchFq.Install (firstServerDevices);
      tchFq.Install (secondServerDevices);
    }
  tchFq.Install (wanLanDevices.Get (1));
  tchFq.Install (pingClientDevices);
  if (dceReceiver)
    {
      // omit device (1) since it is DCE
      tchFq.Install (firstClientDevices.Get (0));
      tchFq.Install (secondClientDevices.Get (0));
    }
  else
    {
      tchFq.Install (firstClientDevices);
      tchFq.Install (secondClientDevices);
    }
  // Install queue for bottleneck link
  TrafficControlHelper tchBottleneck;
  tchBottleneck.SetRootQueueDisc (queueTypeId.GetName ());
  tchBottleneck.SetQueueLimits ("ns3::DynamicQueueLimits", "HoldTime", StringValue ("1ms"));
  tchBottleneck.Install (wanLanDevices.Get (0));

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer pingServerIfaces = ipv4.Assign (pingServerDevices);
  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer firstServerIfaces = ipv4.Assign (firstServerDevices);
  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer secondServerIfaces = ipv4.Assign (secondServerDevices);
  ipv4.SetBase ("172.16.1.0", "255.255.255.0");
  Ipv4InterfaceContainer wanLanIfaces = ipv4.Assign (wanLanDevices);
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
  // Trace drops and marks for bottleneck
  tc = wanLanDevices.Get (0)->GetNode ()->GetObject<TrafficControlLayer> ();
  qd = tc->GetRootQueueDiscOnDevice (wanLanDevices.Get (0));
  qd->TraceConnectWithoutContext ("Mark", MakeBoundCallback (&TraceQueueMark, &queueMarkOfStream));
  qd->TraceConnectWithoutContext ("BytesInQueue", MakeBoundCallback (&TraceQueueLength, &queueLengthOfStream, linkRate));

  // Setup scheduled traces; TCP traces must be hooked after socket creation
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstTcpRttTraceConnection, &firstTcpRttOfStream);
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstTcpCwndTraceConnection, &firstTcpCwndOfStream);
  Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstPacketSinkConnection);
  if (firstTcpType == "dctcp")
    {
      Simulator::Schedule (Seconds (5) + MilliSeconds (100), &ScheduleFirstDctcpTraceConnection, &firstTcpDctcpOfStream);
    }
  Simulator::Schedule (throughputSamplingInterval, &TraceFirstThroughput, &firstTcpThroughputOfStream, throughputSamplingInterval);
#if 0
  // Setup scheduled traces; TCP traces must be hooked after socket creation
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondTcpRttTraceConnection, &secondTcpRttOfStream);
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondTcpCwndTraceConnection, &secondTcpCwndOfStream);
  Simulator::Schedule (Seconds (15) + MilliSeconds (100), &ScheduleSecondPacketSinkConnection);
#endif
  Simulator::Schedule (throughputSamplingInterval, &TraceSecondThroughput, &secondTcpThroughputOfStream, throughputSamplingInterval);
  Simulator::Schedule (marksSamplingInterval, &TraceMarksFrequency, &queueMarksFrequencyOfStream, marksSamplingInterval);

  if (enablePcap)
    {
      p2p.EnablePcapAll ("tcp-validation", false);
    }

  Simulator::Stop (stopTime);
  Simulator::Run ();

  pingOfStream.close ();
  firstTcpCwndOfStream.close ();
  firstTcpRttOfStream.close ();
  if (firstTcpType == "dctcp")
    {
      firstTcpDctcpOfStream.close ();
    }
  firstTcpThroughputOfStream.close ();
  secondTcpCwndOfStream.close ();
  secondTcpRttOfStream.close ();
  secondTcpThroughputOfStream.close ();
  queueMarkOfStream.close ();
  queueMarksFrequencyOfStream.close ();
  queueLengthOfStream.close ();
}

