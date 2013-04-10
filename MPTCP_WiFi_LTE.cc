#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdint.h>
#include <cassert>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/csma-module.h"

//#include "ns3/simulator-module.h"
//#include "ns3/node-module.h"
//#include "ns3/helper-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/lte-module.h"
#include "ns3/global-route-manager.h"

#include "ns3/mp-internet-stack-helper.h"
#include "ns3/mp-tcp-packet-sink.h"
#include "ns3/mp-tcp-l4-protocol.h"
#include "ns3/mp-tcp-socket-base.h"
#include "ns3/mp-tcp-packet-sink-helper.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-net-device.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FirstMultipathToplogy");

// The number of bytes to send in this simulation.
static const uint32_t totalTxBytes = 1000000;
static const uint32_t sendBufSize  = 14000; //2000000;
static const uint32_t recvBufSize  = 2000; //2000000;
static uint32_t currentTxBytes     = 0;
static const double simDuration    = 20.0;

static const uint32_t nSpokes = 2;
static const uint32_t nWiFi = 1;

Ptr<Node> client;
Ptr<Node> server;


// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = sendBufSize;
uint8_t data[totalTxBytes];
Ptr<MpTcpSocketBase> lSocket    = 0;

void StartFlow (Ptr<MpTcpSocketBase>, Ipv4Address, uint16_t);
void WriteUntilBufferFull (Ptr<Socket>, unsigned int);
void connectionSucceeded(Ptr<Socket>);
void connectionFailed(Ptr<Socket>);

void HandlePeerClose (Ptr<Socket>);
void HandlePeerError (Ptr<Socket>);
void CloseConnection (Ptr<Socket>);

int connect(Address &addr);
void variateDelay(PointToPointHelper P2Plink);

static void
CwndTracer (double oldval, double newval)
{
  NS_LOG_INFO ("Moving cwnd from " << oldval << " to " << newval);
}


int main(int argc, char *argv[])
{
  bool verbose;
  CongestionCtrl_t cc = Fully_Coupled;
  PacketReorder_t  pr = D_SACK;
  int arg1 = -1, arg2 = -1, arg3 = -1, arg4 = -1;
  int sf = 2; // number of subflows
  CommandLine cmd;

  cmd.AddValue("verbose", "Tell application to log if true", verbose);
  cmd.AddValue("level", "Tell application which log level to use:\n \t - 0 = ERROR \n \t - 1 = WARN \n \t - 2 = DEBUG \n \t - 3 = INFO \n \t - 4 = FUNCTION \n \t - 5 = LOGIC \n \t - 6 = ALL", arg3);
  cmd.AddValue("cc", "Tell application which congestion control algorithm to use:\n \t - 0 = Uncoupled_TCPs \n \t - 1 = Linked_Increases \n \t - 2 = RTT_Compensator \n \t - 3 = Fully_Coupled", arg1);
  cmd.AddValue("pr", "Tell application which packet reordering algorithm to use:\n \t - 0 = NoPR_Algo \n \t - 1 = Eifel \n \t - 2 = TCP_DOOR \n \t - 3 = D_SACK \n \t - 4 = F_RTO",  arg2);
  cmd.AddValue("sf", "Tell application the number of subflows to be established between endpoints",  arg4);

  cmd.Parse (argc, argv);

  cc = (arg1==-1 ? Fully_Coupled:(CongestionCtrl_t) arg1);
  pr = (arg2==-1 ? D_SACK:(PacketReorder_t) arg2);
  sf = (arg4 = -1 ? 2: arg4);

//  Log running events
//  LogComponentEnable("FirstMultipathToplogy", LOG_LEVEL_ALL);
//  LogComponentEnable("MpTcpSocketBase", LOG_LEVEL_ALL);
//  LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("Icmpv6L4Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("Icmpv4L4Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("MpTcpL4Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("Packet", LOG_LEVEL_ALL);
//  LogComponentEnable("Socket", LOG_LEVEL_ALL);
//  LogComponentEnable ("PointToPointNetDevice", LOG_ALL);
//  LogComponentEnable ("MpTcpHeader", LOG_WARN);
//  LogComponentEnable ("MpTcpL4Protocol", LOG_ALL);
//  LogComponentEnable("Simulator",LOG_LEVEL_ALL);
//  LogComponentEnable("TcpHeader", LOG_LEVEL_ALL);
//  LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);
//  LogComponentEnable("MpTcpTypeDefs", LOG_LEVEL_ALL);
//  LogComponentEnable("RttEstimator", LOG_LEVEL_ALL);
  if(arg3 == 2)
    LogComponentEnable("MpTcpSocketBase", LOG_DEBUG);
  else if(arg3 == 6)
    LogComponentEnable("MpTcpSocketBase", LOG_LEVEL_ALL);
  else
    LogComponentEnable("MpTcpSocketBase", LOG_LEVEL_WARN);


    // A. Creation of network topology

/* Multipath Network Topology

         WiFi 10.1.1.1  P2P 
        ____ AP ____
	     /            \
10.1.1.3   MS              n2
	     \____ BS ____/
	 LTE  10.1.1.2    P2P
*/

    // A.1. P2P
   
    // Here, we will create N nodes in a star.
    NS_LOG_INFO ("Create P2P nodes.");
    NodeContainer serverNode;
    NodeContainer clientNodes;
    serverNode.Create (1);
    clientNodes.Create (nSpokes);
    NodeContainer p2pNodes = NodeContainer (serverNode, clientNodes);

    // Collect an adjacency list of nodes for the p2p topology
    std::vector<NodeContainer> nodeAdjacencyList (nSpokes);
    for(uint32_t i=0; i<nodeAdjacencyList.size (); ++i)
      {
        nodeAdjacencyList[i] = NodeContainer (serverNode, clientNodes.Get (i));
      }

    // We create the channels first without any IP addressing information
    NS_LOG_INFO ("Create P2P channels.");
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
    p2p.SetChannelAttribute ("Delay", StringValue ("0.1ms"));
    std::vector<NetDeviceContainer> deviceAdjacencyList (nSpokes);
    for(uint32_t i=0; i<deviceAdjacencyList.size (); ++i)
      {
        deviceAdjacencyList[i] = p2p.Install (nodeAdjacencyList[i]);

        p2p.EnablePcap ("EC2M" , clientNodes, true);
      }

    // A.2. WiFi
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create (nWiFi);
    NodeContainer wifiApNode =clientNodes.Get(1);
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
    phy.SetChannel (channel.Create ());

    WifiHelper wifi = WifiHelper::Default ();
    wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

    NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();

    Ssid ssid = Ssid ("ns-3-ssid");
    mac.SetType ("ns3::StaWifiMac",
                 "Ssid", SsidValue (ssid),
                 "ActiveProbing", BooleanValue (false));

    NetDeviceContainer staDevices;
    staDevices = wifi.Install (phy, mac, wifiStaNodes);

    mac.SetType ("ns3::ApWifiMac",
                 "Ssid", SsidValue (ssid));

    NetDeviceContainer apDevices;
    apDevices = wifi.Install (phy, mac, wifiApNode);

    MobilityHelper mobility;

    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (5.0),
                                   "DeltaY", DoubleValue (10.0),
                                   "GridWidth", UintegerValue (3),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
    mobility.Install (wifiStaNodes);

    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifiApNode);
   
    // A.3. LTE
    // CREATE NODE CONTAINER AND CREATE LTE NODES
    LteHelper lte;
    NodeContainer ueNodes = wifiStaNodes.Get(0);
    //for (int i=0; i < nWiFi; i++)
    //{
    //ueNodes.Add(wifiStaNodes.Get(i));
    //}
    NodeContainer enbNodes = clientNodes.Get(1);

    // CREATE DEVICE CONTAINER, INSTALL DEVICE TO NODE
    NetDeviceContainer ueDevs, enbDevs;
    ueDevs = lte.Install (ueNodes, LteHelper::DEVICE_TYPE_USER_EQUIPMENT);
    enbDevs = lte.Install (enbNodes, LteHelper::DEVICE_TYPE_ENODEB);

    // MANAGE LTE NET DEVICES
    Ptr<EnbNetDevice> enb;
    enb = enbDevs.Get (0)->GetObject<EnbNetDevice> ();

    std::vector<Ptr<UeNetDevice> > ue (1);
    for (int i = 0; i < 1; i++)
      {
        ue.at (i) = ueDevs.Get (i)->GetObject<UeNetDevice> ();
        lte.RegisterUeToTheEnb (ue.at (i), enb);
      }
    // CONFIGURE DL and UL SUB CHANNELS
    // Define a list of sub channels for the downlink
    std::vector<int> dlSubChannels;
    for (int i = 0; i < 25; i++)
      {
        dlSubChannels.push_back (i);
      }
    // Define a list of sub channels for the uplink
    std::vector<int> ulSubChannels;
    for (int i = 50; i < 100; i++)
      {
        ulSubChannels.push_back (i);
      }

    enb->GetPhy ()->SetDownlinkSubChannels (dlSubChannels);
    enb->GetPhy ()->SetUplinkSubChannels (ulSubChannels);

    for (int i = 0; i < 1; i++)
      {
        ue.at (i)->GetPhy ()->SetDownlinkSubChannels (dlSubChannels);
        ue.at (i)->GetPhy ()->SetUplinkSubChannels (ulSubChannels);
      }


    // CONFIGURE MOBILITY
    Ptr<ConstantPositionMobilityModel> enbMobility = CreateObject<ConstantPositionMobilityModel> ();
    enbMobility->SetPosition (Vector (0.0, 0.0, 0.0));
    lte.AddMobility (enb->GetPhy (), enbMobility);

    for (int i = 0; i < 1; i++)
      {
        Ptr<ConstantVelocityMobilityModel> ueMobility = CreateObject<ConstantVelocityMobilityModel> ();
        ueMobility->SetPosition (Vector (30.0, 0.0, 0.0));
        ueMobility->SetVelocity (Vector (30.0, 0.0, 0.0));
  
        lte.AddMobility (ue.at (i)->GetPhy (), ueMobility);

        lte.AddDownlinkChannelRealization (enbMobility, ueMobility, ue.at (i)->GetPhy ());
      }


    // A.4. MPTCP
    client = wifiStaNodes.Get(0);
    server = serverNode.Get(0);

    // B IP addressing.

    // Install network stacks on the nodes
    InternetStackHelper internet;
    internet.SetTcp ("ns3::MpTcpL4Protocol");
    internet.Install (p2pNodes);

    //internet.Install (wifiApNode);
    internet.Install (wifiStaNodes);
 
    //internet.Install (ueNodes);
    //internet.Install (enbNodes);

    NS_LOG_INFO ("Assign IP Addresses.");
    
    // B.1 P2P IP
    Ipv4AddressHelper ipv4;
    std::vector<Ipv4InterfaceContainer> interfaceAdjacencyList (nSpokes);
    for(uint32_t i=0; i<interfaceAdjacencyList.size (); ++i)
      {
        std::ostringstream subnet;
        subnet<<"10.1."<<i+1<<".0";
        ipv4.SetBase (subnet.str ().c_str (), "255.255.255.0");
        interfaceAdjacencyList[i] = ipv4.Assign (deviceAdjacencyList[i]);
      }

    // B.2 WiFi IP
    ipv4.SetBase ("10.1.3.0", "255.255.255.0");
    ipv4.Assign(staDevices);
    ipv4.Assign(apDevices);
    

    // B.3 Lte IP
    ipv4.SetBase ("10.1.4.0", "255.255.255.0");
    ipv4.Assign(ueDevs);
    ipv4.Assign(enbDevs);
    
    //Turn on global static routing
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

/////////////////////////////////////////////
/*
    uint32_t servPort = 5000;
    NS_LOG_INFO ("address " << interfaceAdjacencyList[0].GetAddress (1));

//    uint32_t servPort = 5000;
//    NS_LOG_INFO ("address " << ipv4Ints[0].GetAddress (1));
//    ObjectFactory m_sf;
//
//    m_sf.SetTypeId("ns3::MpTcpPacketSink");
//    m_sf.Set("Protocol", StringValue ("ns3::TcpSocketFactoryImpl"));
//    m_sf.Set("Local", AddressValue(InetSocketAddress (ipv4Ints[0].GetAddress (1), servPort)));
//    m_sf.Set("algopr", UintegerValue ((uint32_t) pr));
//
//    Ptr<Application> sapp = m_sf.Create<Application> ();
//    server->AddApplication(sapp);

  MpTcpPacketSinkHelper help("ns3::TcpSocketFactory",(InetSocketAddress (interfaceAdjacencyList[0].GetAddress (1), servPort)), pr);
  ApplicationContainer sapp=help.Install(server);
//////////////////////////////////////////////////////////////

    ApplicationContainer Apps;
    Apps.Add(sapp);
//    Apps.Add(capp);


    //ApplicationContainer serverApps = sink.Install(server);
    Apps.Start(Seconds(0.0));
    Apps.Stop(Seconds(simDuration));


    //Ptr<Socket> localSocket = Socket::CreateSocket(client, TcpSocketFactory::GetTypeId());
    lSocket = new MpTcpSocketBase (client);
    
    //lSocket->SetCongestionCtrlAlgo (Linked_Increases);
    //lSocket->SetCongestionCtrlAlgo (RTT_Compensator);
    lSocket->SetCongestionCtrlAlgo (cc);
    //lSocket->SetCongestionCtrlAlgo (Uncoupled_TCPs);

    lSocket->SetDataDistribAlgo (Round_Robin);

    //lSocket->SetPacketReorderAlgo (Eifel);
    lSocket->SetPacketReorderAlgo (pr);

    lSocket->Bind ();

    // Trace changes to the congestion window
    Config::ConnectWithoutContext ("/NodeList/0/$ns3::MpTcpSocketBase/subflows/0/CongestionWindow", MakeCallback (&CwndTracer));

    // ...and schedule the sending "Application"; This is similar to what an
    // ns3::Application subclass would do internally.
    Simulator::ScheduleNow (&StartFlow, lSocket, interfaceAdjacencyList[0].GetAddress (1), servPort);
*/

    // Finally, set up the simulator to run.  The 1000 second hard limit is a
    // failsafe in case some change above causes the simulation to never end
    Simulator::Stop (Seconds(simDuration + 1000.0));
    Simulator::Run ();
    Simulator::Destroy();

NS_LOG_LOGIC("mpTopology:: simulation ended");
    return 0;
}


//begin implementation of sending "Application"
void StartFlow(Ptr<MpTcpSocketBase> localSocket, Ipv4Address servAddress, uint16_t servPort)
{
  NS_LOG_LOGIC("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
  //localSocket->Connect (InetSocketAddress (servAddress, servPort));//connect
  lSocket->SetMaxSubFlowNumber(5);
  lSocket->SetMinSubFlowNumber(3);
  lSocket->SetSourceAddress(Ipv4Address("10.1.1.1"));
  lSocket->allocateSendingBuffer(sendBufSize);
  lSocket->allocateRecvingBuffer(recvBufSize);
  // the following buffer is uesed by the received to hold out of sequence data
  lSocket->SetunOrdBufMaxSize(50);


  int connectionState = lSocket->Connect( servAddress, servPort);
//NS_LOG_LOGIC("mpTopology:: connection request sent");
  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available

  if(connectionState == 0)
  {
      lSocket->SetConnectCallback  (MakeCallback (&connectionSucceeded), MakeCallback (&connectionFailed));
      lSocket->SetDataSentCallback (MakeCallback (&WriteUntilBufferFull));
      lSocket->SetCloseCallbacks   (MakeCallback (&HandlePeerClose), MakeCallback(&HandlePeerError));
      lSocket->GetSubflow(0)->StartTracing ("CongestionWindow");
  }else
  {
      //localSocket->NotifyConnectionFailed();
      NS_LOG_LOGIC("mpTopology:: connection failed");
  }
  //WriteUntilBufferFull (localSocket);
}

void connectionSucceeded (Ptr<Socket> localSocket)
{
    NS_LOG_FUNCTION_NOARGS();
    NS_LOG_INFO("mpTopology: Connection requeste succeed");
    Simulator::Schedule (Seconds (1.0), &WriteUntilBufferFull, lSocket, 0);
    Simulator::Schedule (Seconds (simDuration), &CloseConnection, lSocket);
    //Ptr<MpTcpSocketImpl> lSock = localSocket;
    // advertise local addresses
    //lSocket->InitiateSubflows();
    //WriteUntilBufferFull(lSocket, 0);
}

void connectionFailed (Ptr<Socket> localSocket)
{
    NS_LOG_FUNCTION_NOARGS();
    NS_LOG_INFO("mpTopology: Connection requeste failure");
    lSocket->Close();
}

void HandlePeerClose (Ptr<Socket> localSocket)
{
    NS_LOG_FUNCTION_NOARGS();
    NS_LOG_INFO("mpTopology: Connection closed by peer");
    lSocket->Close();
}

void HandlePeerError (Ptr<Socket> localSocket)
{
    NS_LOG_FUNCTION_NOARGS();
    NS_LOG_INFO("mpTopology: Connection closed by peer error");
    lSocket->Close();
}

void CloseConnection (Ptr<Socket> localSocket)
{
    lSocket->Close();
    NS_LOG_LOGIC("mpTopology:: currentTxBytes = " << currentTxBytes);
    NS_LOG_LOGIC("mpTopology:: totalTxBytes   = " << totalTxBytes);
    NS_LOG_LOGIC("mpTopology:: connection to remote host has been closed");
}

void variateDelay (Ptr<Node> node)
{
    //NS_LOG_INFO ("variateDelay");

    Ptr<Ipv4L3Protocol> ipv4 = node->GetObject<Ipv4L3Protocol> ();
    TimeValue delay;
    for(uint32_t i = 0; i < ipv4->GetNInterfaces(); i++)
    {
        //Ptr<NetDevice> device = m_node->GetDevice(i);
        Ptr<Ipv4Interface> interface = ipv4->GetInterface(i);
        Ipv4InterfaceAddress interfaceAddr = interface->GetAddress (0);
        // do not consider loopback addresses
        if(interfaceAddr.GetLocal() == Ipv4Address::GetLoopback())
        {
            // loopback interface has identifier equal to zero
            continue;
        }

        Ptr<NetDevice> netDev =  interface->GetDevice();
        Ptr<Channel> P2Plink  =  netDev->GetChannel();
        P2Plink->GetAttribute(string("Delay"), delay);
        double oldDelay = delay.Get().GetSeconds();
        //NS_LOG_INFO ("variateDelay -> old delay == " << oldDelay);
        std::stringstream strDelay;
        double newDelay = (rand() % 100) * 0.001;
        double err = newDelay - oldDelay;
        strDelay << (0.95 * oldDelay + 0.05 * err) << "s";
        P2Plink->SetAttribute(string("Delay"), StringValue(strDelay.str()));
        P2Plink->GetAttribute(string("Delay"), delay);
        //NS_LOG_INFO ("variateDelay -> new delay == " << delay.Get().GetSeconds());
    }
}

void WriteUntilBufferFull (Ptr<Socket> localSocket, unsigned int txSpace)
{
    NS_LOG_FUNCTION_NOARGS();

  while (currentTxBytes < totalTxBytes && lSocket->GetTxAvailable () > 0)
  {
      uint32_t left    = totalTxBytes - currentTxBytes;
      uint32_t toWrite = std::min(writeSize, lSocket->GetTxAvailable ());
      toWrite = std::min( toWrite , left );

      int amountBuffered = lSocket->FillBuffer (&data[currentTxBytes], toWrite);
      currentTxBytes += amountBuffered;

      variateDelay(client);

      lSocket->SendBufferedData();
  }

}

