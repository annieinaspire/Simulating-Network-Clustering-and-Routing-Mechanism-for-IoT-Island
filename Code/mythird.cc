#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-radio-energy-model.h"
#include <stdint.h>
#include "ns3/buffer.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/olsr-helper.h"
//#include "ns3/wifi-module.h"
//#include "ns3/assert.h"
//#include "ns3/log.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("WirelessAnimationExample");
///////buffer in sink
static const uint32_t buffersize = 20400000;

int i=0;
int k=2;
uint32_t packetsize=20;
double deltaTime = 1;
double averageTx[7];
double totalTx[7];
uint32_t size[7];
uint32_t revPacket[7];//record the amount of packets received by RP
uint8_t buffer[7][buffersize];//for two access points (RP)
uint8_t* pointer[7]={buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]};
//first two for 3 apbuffer, and 4 for msbuffer

uint32_t uploadSize[4];
int uploadPacket[4];//record uploaded packet in upload point
double averageDelay[4];
double totalDelay[4];
int validPacket[4];
class MyTag : public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  void SetSimpleValue (uint8_t value);
  uint8_t GetSimpleValue (void) const;
private:
  uint8_t m_simpleValue;
};

TypeId
MyTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyTag")
    .SetParent<Tag> ()
    .AddConstructor<MyTag> ()
    .AddAttribute ("SimpleValue",
                   "A simple value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&MyTag::GetSimpleValue),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}
TypeId
MyTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
MyTag::GetSerializedSize (void) const
{
  return 1;
}
void
MyTag::Serialize (TagBuffer i) const
{
  i.WriteU8 (m_simpleValue);
}
void
MyTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadU8 ();
}
void
MyTag::Print (std::ostream &os) const
{
  os << "v=" << (uint32_t)m_simpleValue;
}
void
MyTag::SetSimpleValue (uint8_t value)
{
  m_simpleValue = value;
}
uint8_t
MyTag::GetSimpleValue (void) const
{
  return m_simpleValue;
}
void sendpacket(Ptr<Socket> sender, InetSocketAddress address) {
	uint32_t num= sender->GetNode()->GetId();
	 Packet sendPacket= Packet(buffer[num],size[num]);
	MyTag tag;
	 tag.SetSimpleValue (averageTx[num]);
   std::cout<<"now the time is "<<averageTx[num]<<std::endl;
	    // store the tag in a packet.
	    sendPacket.AddPacketTag (tag);
		    sender->Connect (address);
		    sender->Send(&sendPacket,0);
	//sender->SendTo(buffer[num],size[num],0,address);
	//std::cout<<"node "<<num <<"send a packet with size "<<length<<"  "<<size[num]<<std::endl;
	pointer[num]=buffer[num];
	size[num]=0;
	averageTx[num]=0;

}
//void forwardpacket(Ptr<Socket> sender, InetSocketAddress address) {
//	uint32_t num= sender->GetNode()->GetId();
//	 Packet sendPacket= Packet(buffer[num],size[num]);
//
//	    // store the tag in a packet.
//		    sender->Connect (address);
//		    sender->Send(&sendPacket,0);
//	//sender->SendTo(buffer[num],size[num],0,address);
//	//std::cout<<"node "<<num <<"send a packet with size "<<length<<"  "<<size[num]<<std::endl;
//	pointer[num]=buffer[num];
//	size[num]=0;
//
//}

void showPosition (Ptr<Node> node,Ptr<Socket>sender,Ipv4InterfaceContainer msInterfaces,Ipv4InterfaceContainer upInterfaces,Ptr<Socket> recvSink[3])
{
  uint32_t nodeId = node->GetId ();
  Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel> ();
  Vector3D pos = mobModel->GetPosition ();
 // Vector3D speed = mobModel->GetVelocity ();
 // if(pos.x > 249 && pos.x < 251 && pos.y > 749 && pos.y < 751){
//  std::cout << "QAQ At " << Simulator::Now ().GetSeconds () << " node " << nodeId
//            << ": Position(" << pos.x << ", " << pos.y << ", " << pos.z
//            << std::endl;

  if(pos.x>=498 &&pos.x<=502 &&pos.y>=498 && pos.y<=502){//mobile sink get to upload point 0
	 // Ptr<Socket>
	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> Mobile sink "<<nodeId<< " upload data to Upload Point 0 ";
	  std::cout << "at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
	 InetSocketAddress  sendaddress = InetSocketAddress (upInterfaces.GetAddress (0), 9);
	 sendpacket(sender,sendaddress);
  }

  if(pos.x >= 248 && pos.x <= 252 && pos.y>=748 && pos.y<=752){//mobile sink get to upload point 1
  	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"-->Mobile sink "<<nodeId<< " uploads data to Upload Point 1" ;
  	std::cout << " at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
  	 InetSocketAddress  sendaddress = InetSocketAddress (upInterfaces.GetAddress (1), 9);
  	sendpacket(sender,sendaddress);
    }
  if(pos.x>=998 && pos.x <=1002 &&pos.y>=348 && pos.y<=352){//mobile sink get to upload point 0
  	 // Ptr<Socket>
  	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> Mobile sink "<<nodeId<< " upload data to Upload Point 2 ";
  	  std::cout << " at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
  	 InetSocketAddress  sendaddress = InetSocketAddress (upInterfaces.GetAddress (2), 9);
  	sendpacket(sender,sendaddress);
    }
  if(pos.x>=998 && pos.x <=1002 &&pos.y>=998 && pos.y <=1002){//mobile sink get to upload point 1
   	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> Mobile sink "<<nodeId<< " uploads data to Upload Point 3";
   	  std::cout << " at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
   	  InetSocketAddress  sendaddress = InetSocketAddress (upInterfaces.GetAddress (3), 9);
        sendpacket(sender,sendaddress);
     }
  if(pos.x>=248 && pos.x <=252 && pos.y>=348 && pos.y<=352){//mobile sink get to RP0
    	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> RP 0 sents data to Mobile Sink " <<nodeId;
    	  InetSocketAddress  sendaddress = InetSocketAddress (msInterfaces.GetAddress (nodeId), 9);
    	  std::cout << " at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
    	  sendpacket(recvSink[0],sendaddress);
      }
  if(pos.x>=373 && pos.x <=377 &&pos.y<=626 && pos.y>=623 ){//mobile sink get to RP0
     	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> RP 1"<< " sents data to Mobile Sink " <<nodeId;
     	  std::cout << " at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )" << std::endl;
     	 InetSocketAddress  sendaddress = InetSocketAddress (msInterfaces.GetAddress (nodeId), 9);
     	  sendpacket(recvSink[1],sendaddress);
       }
  if(pos.x>=623&& pos.x <=627 &&pos.y>=623 &&pos.y<=627){//mobile sink get to RP0
     	  std::cout <<"\n Time: "<< Simulator::Now ().GetSeconds ()<<" --> RP 2 sents data to Mobile Sink " <<nodeId;
     	  InetSocketAddress  sendaddress = InetSocketAddress (msInterfaces.GetAddress (nodeId), 9);
     	  std::cout << "at " << "( " << pos.x << ", " << pos.y << ", " << pos.z<<" )"<< std::endl;
     	  sendpacket(recvSink[2],sendaddress);
       }

  Simulator::ScheduleWithContext (nodeId,Seconds (deltaTime), &showPosition, node,sender,msInterfaces,upInterfaces,recvSink);
}


void Upload(Ptr<Socket> socket){
	Ptr<Packet> packet;

	uint32_t num= socket->GetNode()->GetId();
	num=num-7;
	while((packet=socket->Recv())){
    MyTag tagCopy;
    packet->PeekPacketTag (tagCopy);
   uint32_t length2= packet->GetSize();
    double time =tagCopy.GetSimpleValue();
   // std::cout<<"!!!finally ---"<<time <<std::endl;
    if(packet->GetSize()!=0) {
    	validPacket[num]=validPacket[num]+length2;
    //	double delay=Simulator::Now().GetSeconds()-time;
    	totalDelay[num]=totalDelay[num]+(Simulator::Now().GetSeconds()-time)*length2;
    	//std::cout<<"!!!!! total Delay "<< totalDelay[num];
    	averageDelay[num]=totalDelay[num]/validPacket[num];
    //	std::cout<<"average delay is "<<averageDelay[num]<<std::endl;
    	 uploadPacket[num]++;
    }
	uint32_t length=packet->GetSize();
	   uploadSize[num]=uploadSize[num]+length;
	 //  uploadPacket[num]++;
	std::cout<<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> Upload Point " << num << " receive a packet with size "<<length<<std::endl;
	//std::cout<<"packet size is "<<uploadPacket[num]<<" and "<<num<<std::endl;
}
}
void ReceivePacket (Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	uint32_t num= socket->GetNode()->GetId();
//packet->GetSize();
  while ((packet=socket->Recv()))//socket->Recv(buffer,500,0)
    {
	  SeqTsHeader seqTs;
	  	  packet->RemoveHeader (seqTs);
	  	 totalTx[num]=totalTx[num]+ seqTs.GetTs().GetSeconds();
	  	revPacket[num]++;
	  	averageTx[num]=totalTx[num]/revPacket[num];
//////////////////////////////
	  //Address addr;
	//  socket->GetSockName(addr);
	  /*InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr);
	  std::cout << "Received one packet!  Socket: " << iaddr.GetIpv4 () << " port: " << iaddr.GetPort ();
      packet->CopyData(pointer[num],packetsize);
      std::cout<<"access point receive a packet with size "<<packet->GetSize()<<" "<<std::endl;*/
      size[num]=size[num]+(packet->GetSize());
     // revPacket[num]++;
      /*std::cout<<size[num]<<":: "<<std::endl;*/
      pointer[num]=pointer[num]+(packet->GetSize());
    //  packet->EnablePrinting();
     // Tag *tag = dynamic_cast<Tag *> (constructor ());
     // Header heard;
     // packet->Print(std::cout);
     // uint32_t uid = packet->GetUid ();
     // packet->GetByteTagIterator()
   //   std::cout<<uid<<""<<std::endl;
     // NS_LOG_UNCOND ("Received one packet! I want write it into buffer !");
    // std::cout<<*pointer<<","<<" ";

    }

}
void msReceivePacket (Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	uint32_t num= socket->GetNode()->GetId();
//packet->GetSize();
  while ((packet=socket->Recv()))//socket->Recv(buffer,500,0)
    {
	  Address addr;
	  socket->GetSockName(addr);
	// InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (addr);
	 //   std::cout << "Received one packet!  Socket: " << iaddr.GetIpv4 () << " port: " << iaddr.GetPort ();
      packet->CopyData(pointer[num],packetsize);
      MyTag tagCopy;
         packet->PeekPacketTag (tagCopy);
         double time =tagCopy.GetSimpleValue();
      //   std::cout<<"!!!now the time is !!!"<<time<<std::endl;
      std::cout<<"\n Time: "<< Simulator::Now ().GetSeconds ()<<"--> Mobile Sink "<<num<<" receive a packet with size "<<packet->GetSize()<<" "<<std::endl;
      averageTx[num]=time;
      size[num]=size[num]+(packet->GetSize());
      revPacket[num]++;
    //  std::cout<<size[num]<<":: ";
      pointer[num]=pointer[num]+(packet->GetSize());

    }
//  Packet sendPacket= Packet(buffer[num],size[num]);
//
//  uint32_t length= sendPacket.GetSize();
//   std::cout<<"mobile sink "<<length<<"->_>-->"<<" "<<size[num]<<std::endl;
//   sendPacket.EnablePrinting();
//  sendPacket.Print(std::cout);
}


int 
main (int argc, char *argv[])
{
	LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);
	bool tracing=true;
//	bool verbose=true;
	double distance=20;
    uint32_t nWifi = 20;
    CommandLine cmd;
    cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);

 cmd.Parse (argc,argv);

 std::string traceFile = "scratch/bonnmotion4.ns_movements";
//////////////////mobile sink node
 NodeContainer mobileSinks;
 mobileSinks.Create(4);
 Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install ();

 NodeContainer allNodes;
 NodeContainer wifiApNodes ;
   wifiApNodes.Create (3);
 NodeContainer uploadPoints;
  uploadPoints.Create(4);
  NodeContainer wifiStaNodes;
 wifiStaNodes.Create (nWifi);
 NodeContainer wifiStaNodes2;
 wifiStaNodes2.Create (nWifi);
 NodeContainer wifiStaNodes3;
 wifiStaNodes3.Create (nWifi);
 allNodes.Add (wifiStaNodes);
 allNodes.Add (wifiStaNodes2);
 allNodes.Add (wifiStaNodes3);
  allNodes.Add (wifiApNodes);
 allNodes.Add(mobileSinks);
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());
  phy.Set ("EnergyDetectionThreshold", DoubleValue (-206.0) );//96
     phy.Set ("CcaMode1Threshold", DoubleValue (-99.0) );
     phy.Set ("TxGain", DoubleValue (4.5) );
     phy.Set ("RxGain", DoubleValue (4.5) );
     phy.Set ("TxPowerLevels", UintegerValue (1) );
     phy.Set ("TxPowerEnd", DoubleValue (16) );
     phy.Set ("TxPowerStart", DoubleValue (16) );
     phy.Set ("RxNoiseFigure", DoubleValue (4) );
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
 
  NqosWifiMacHelper mac;
  Ssid ssid = Ssid ("ns-3-ssid");
//  mac.SetType ("ns3::AdhocWifiMac");
//  mac.SetType ("ns3::StaWifiMac",
// "Ssid", SsidValue (ssid),
//  "ActiveProbing", BooleanValue (false));
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer apDevices0;
   apDevices0 = wifi.Install (phy, mac, wifiApNodes.Get(0));
 NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);
  ////
  YansWifiChannelHelper channel1 = YansWifiChannelHelper::Default ();
        YansWifiPhyHelper phy1 = YansWifiPhyHelper::Default ();
        phy1.SetChannel (channel1.Create ());
        phy1.Set ("EnergyDetectionThreshold", DoubleValue (-206.0) );//96
           phy1.Set ("CcaMode1Threshold", DoubleValue (-99.0) );
           phy1.Set ("TxGain", DoubleValue (4.5) );
           phy1.Set ("RxGain", DoubleValue (4.5) );
           phy1.Set ("TxPowerLevels", UintegerValue (1) );
           phy1.Set ("TxPowerEnd", DoubleValue (16) );
           phy1.Set ("TxPowerStart", DoubleValue (16) );
           phy1.Set ("RxNoiseFigure", DoubleValue (4) );
           /////
  NetDeviceContainer staDevices2;
  staDevices2 = wifi.Install (phy1, mac, wifiStaNodes2);
  NetDeviceContainer apDevices1;
      apDevices1 = wifi.Install (phy1, mac, wifiApNodes.Get(1));
      ///////
      YansWifiChannelHelper channel2 = YansWifiChannelHelper::Default ();
                 YansWifiPhyHelper phy2 = YansWifiPhyHelper::Default ();
                 phy2.SetChannel (channel2.Create ());
                 phy2.Set ("EnergyDetectionThreshold", DoubleValue (-206.0) );//96
                    phy2.Set ("CcaMode1Threshold", DoubleValue (-99.0) );
                    phy2.Set ("TxGain", DoubleValue (4.5) );
                    phy2.Set ("RxGain", DoubleValue (4.5) );
                    phy2.Set ("TxPowerLevels", UintegerValue (1) );
                    phy2.Set ("TxPowerEnd", DoubleValue (16) );
                    phy2.Set ("TxPowerStart", DoubleValue (16) );
                    phy2.Set ("RxNoiseFigure", DoubleValue (4) );
                    /////////
  NetDeviceContainer staDevices3;
  staDevices3 = wifi.Install (phy2, mac, wifiStaNodes3);
  NetDeviceContainer apDevices2;
      apDevices2 = wifi.Install (phy2, mac, wifiApNodes.Get(2));
      YansWifiChannelHelper channel3 = YansWifiChannelHelper::Default ();
      YansWifiPhyHelper phy3 = YansWifiPhyHelper::Default ();
      phy3.SetChannel (channel3.Create ());
      phy3.Set ("EnergyDetectionThreshold", DoubleValue (-206.0) );//96
      phy3.Set ("CcaMode1Threshold", DoubleValue (-99.0) );
      phy3.Set ("TxGain", DoubleValue (4.5) );
      phy3.Set ("RxGain", DoubleValue (4.5) );
      phy3.Set ("TxPowerLevels", UintegerValue (1) );
      phy3.Set ("TxPowerEnd", DoubleValue (16) );
      phy3.Set ("TxPowerStart", DoubleValue (16) );
      phy3.Set ("RxNoiseFigure", DoubleValue (4) );

 allNodes.Add(uploadPoints);
 NetDeviceContainer upDevices;
 NodeContainer forwarders;
  forwarders.Add(wifiApNodes);
 upDevices= wifi.Install (phy3, mac, uploadPoints);//upload point is also a sink
    NetDeviceContainer msDevices;
  msDevices = wifi.Install (phy3, mac, mobileSinks);
  NetDeviceContainer fwDevices;
    fwDevices = wifi.Install (phy3, mac, forwarders);

/////////////////////////////////////////////////////////////////////////
  // Mobility
  MobilityHelper mobility;
   mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue (251.0),
                                  "MinY", DoubleValue (251.0),
                                  "DeltaX", DoubleValue (distance),
                                  "DeltaY", DoubleValue (distance),
                                  "GridWidth", UintegerValue (5),
                                  "LayoutType", StringValue ("RowFirst"));
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobility.Install (wifiStaNodes);

   mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue (385.0),
                                  "MinY", DoubleValue (642.0),
                                  "DeltaX", DoubleValue (distance),
                                  "DeltaY", DoubleValue (distance),
                                  "GridWidth", UintegerValue (5),
                                  "LayoutType", StringValue ("RowFirst"));
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobility.Install (wifiStaNodes2);

   mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue (645.0),
                                  "MinY", DoubleValue (645.0),
                                  "DeltaX", DoubleValue (distance),
                                  "DeltaY", DoubleValue (distance),
                                  "GridWidth", UintegerValue (5),
                                  "LayoutType", StringValue ("RowFirst"));
   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
   mobility.Install (wifiStaNodes3);


  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes);
  AnimationInterface::SetConstantPosition (wifiApNodes.Get (0), 250, 350);
  AnimationInterface::SetConstantPosition (wifiApNodes.Get (1), 375, 625);
  AnimationInterface::SetConstantPosition (wifiApNodes.Get (2), 625, 625);
 // AnimationInterface::SetConstantPosition (csmaNodes.Get (1), 500, 500);
  AnimationInterface::SetConstantPosition(uploadPoints.Get(0),500, 500);
  AnimationInterface::SetConstantPosition(uploadPoints.Get(1),250, 750);
  AnimationInterface::SetConstantPosition(uploadPoints.Get(2),1000, 350);
  AnimationInterface::SetConstantPosition(uploadPoints.Get(3),1000, 1000);
 // Ptr<BasicEnergySource> energySource = CreateObject<BasicEnergySource>();
 // Ptr<WifiRadioEnergyModel> energyModel = CreateObject<WifiRadioEnergyModel>();
 
//  energySource->SetInitialEnergy (3);
//  energyModel->SetEnergySource (energySource);
//  energySource->AppendDeviceEnergyModel (energyModel);
//
  // aggregate energy source to node
 // wifiApNodes->AggregateObject (energySource);
 // wifiApNodes.Get (1)->AggregateObject (energySource);
  // Install internet stack

  InternetStackHelper stack;
  stack.Install (allNodes);
  // Install Ipv4 addresses

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer fwInterfaces;
      fwInterfaces = address.Assign (fwDevices);
      Ipv4InterfaceContainer upInterfaces;
        upInterfaces = address.Assign (upDevices);
        Ipv4InterfaceContainer msInterfaces;
         msInterfaces = address.Assign (msDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");

  Ipv4InterfaceContainer staInterfaces;
     staInterfaces = address.Assign (staDevices);
     Ipv4InterfaceContainer apInterface0;
       apInterface0 = address.Assign (apDevices0);

  address.SetBase ("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces2;
    staInterfaces2 = address.Assign (staDevices2);
    Ipv4InterfaceContainer apInterface1;
    apInterface1 = address.Assign (apDevices1);
    address.SetBase ("10.1.4.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces3;
       staInterfaces3 = address.Assign (staDevices3);
       Ipv4InterfaceContainer apInterface2;
           apInterface2 = address.Assign (apDevices2);


  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
 
  Ptr<Socket> recvSink[3];
   InetSocketAddress  local = InetSocketAddress (apInterface0.GetAddress (0), 9);
   InetSocketAddress  local2 = InetSocketAddress (apInterface1.GetAddress (0), 9);
   InetSocketAddress  local3 = InetSocketAddress (apInterface2.GetAddress (0), 9);
   recvSink[0] = Socket::CreateSocket(wifiApNodes.Get(0),tid);
   recvSink[0]->Bind (local);
   recvSink[0]->SetRecvCallback (MakeCallback (&ReceivePacket));
   recvSink[1] = Socket::CreateSocket(wifiApNodes.Get(1),tid);
   recvSink[1]->Bind (local2);
   recvSink[1]->SetRecvCallback (MakeCallback (&ReceivePacket));
   recvSink[2] = Socket::CreateSocket(wifiApNodes.Get(2),tid);
   recvSink[2]->Bind (local3);
   recvSink[2]->SetRecvCallback (MakeCallback (&ReceivePacket));
   ///////////////////////forwarder configuration

       Ptr<Socket>fwSinks[2];
       fwSinks[0]=Socket::CreateSocket(forwarders.Get(0),tid);
          InetSocketAddress  fw0 = InetSocketAddress (fwInterfaces.GetAddress (0), 9);
          fwSinks[0]->Bind(fw0);
         // std::cout<<"!!!"<<fwInterfaces.GetAddress (0)<<std::endl;
          fwSinks[0]->SetRecvCallback (MakeCallback (&msReceivePacket));

	  fwSinks[1]=Socket::CreateSocket(forwarders.Get(1),tid);
          InetSocketAddress  fw1 = InetSocketAddress (fwInterfaces.GetAddress (1), 9);
          fwSinks[1]->Bind(fw1);
          fwSinks[1]->SetRecvCallback (MakeCallback (&msReceivePacket));

	  fwSinks[2]=Socket::CreateSocket(forwarders.Get(2),tid);
          InetSocketAddress  fw2 = InetSocketAddress (fwInterfaces.GetAddress (2), 9);
          fwSinks[2]->Bind(fw2);
          fwSinks[2]->SetRecvCallback (MakeCallback (&msReceivePacket));



///////recvSink also forward buffer data to mobile sink

    /// uploadpoint
      Ptr<Socket>uploadSink[4];
      uploadSink[0] = Socket::CreateSocket(uploadPoints.Get(0),tid);
      InetSocketAddress  up1 = InetSocketAddress (upInterfaces.GetAddress (0), 9);
      uploadSink[0]->Bind(up1);
      uploadSink[1] = Socket::CreateSocket(uploadPoints.Get(1),tid);
      InetSocketAddress  up2 = InetSocketAddress (upInterfaces.GetAddress (1), 9);
      uploadSink[1]->Bind(up2);
      uploadSink[2] = Socket::CreateSocket(uploadPoints.Get(2),tid);
      InetSocketAddress  up3 = InetSocketAddress (upInterfaces.GetAddress (2), 9);
      uploadSink[2]->Bind(up3);
      uploadSink[3] = Socket::CreateSocket(uploadPoints.Get(3),tid);
      InetSocketAddress  up4 = InetSocketAddress (upInterfaces.GetAddress (3), 9);
      uploadSink[3]->Bind(up4);
      uploadSink[0]->SetRecvCallback(MakeCallback(&Upload));
      uploadSink[1]->SetRecvCallback(MakeCallback(&Upload));
      uploadSink[2]->SetRecvCallback(MakeCallback(&Upload));
      uploadSink[3]->SetRecvCallback(MakeCallback(&Upload));



//  ////////////////////////////mobile sink configuration

  Ptr<Socket>mobileSink[4];
   mobileSink[0]=Socket::CreateSocket(mobileSinks.Get(0),tid);
   InetSocketAddress  ms0 = InetSocketAddress (msInterfaces.GetAddress (0), 9);
 //  std::cout<<"!!! mobile sink ip "<<msInterfaces.GetAddress (0)<<std::endl;
   mobileSink[0]->Bind(ms0);
  mobileSink[0]->SetRecvCallback (MakeCallback (&msReceivePacket));
   mobileSink[1]=Socket::CreateSocket(mobileSinks.Get(1),tid);
   InetSocketAddress  ms1 = InetSocketAddress (msInterfaces.GetAddress (1), 9);
  // std::cout<<"!!! mobile sink ip "<<msInterfaces.GetAddress (1)<<std::endl;
   mobileSink[1]->Bind(ms1);
   mobileSink[1]->SetRecvCallback (MakeCallback (&msReceivePacket));
   mobileSink[2]=Socket::CreateSocket(mobileSinks.Get(2),tid);
   InetSocketAddress  ms2 = InetSocketAddress (msInterfaces.GetAddress (2), 9);
  // std::cout<<"!!! mobile sink ip "<<msInterfaces.GetAddress (2)<<std::endl;
   mobileSink[2]->Bind(ms2);
   mobileSink[2]->SetRecvCallback (MakeCallback (&msReceivePacket));
   mobileSink[3]=Socket::CreateSocket(mobileSinks.Get(3),tid);
   InetSocketAddress  ms3 = InetSocketAddress (msInterfaces.GetAddress (3), 9);
  // std::cout<<"!!! mobile sink ip "<<msInterfaces.GetAddress (3)<<std::endl;
   mobileSink[3]->Bind(ms3);
   mobileSink[3]->SetRecvCallback (MakeCallback (&msReceivePacket));
//   /////////////////////////////////////////////////////////////station node configuration


  UdpClientHelper echoClient (apInterface0.GetAddress (0), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (100));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (5.)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (packetsize));
  ApplicationContainer clientApps = echoClient.Install (wifiStaNodes);
  clientApps.Start (Seconds (1.0));
  clientApps.Stop (Seconds (1000.0));

  UdpClientHelper echoClient2 (apInterface1.GetAddress (0), 9);
  echoClient2.SetAttribute ("MaxPackets", UintegerValue (20));
  echoClient2.SetAttribute ("Interval", TimeValue (Seconds (5.)));
  echoClient2.SetAttribute ("PacketSize", UintegerValue (packetsize));
  ApplicationContainer clientApps2 = echoClient2.Install (wifiStaNodes2);
  clientApps2.Start (Seconds (1.0));
  clientApps2.Stop (Seconds (1000.0));

  UdpClientHelper echoClient3 (apInterface2.GetAddress (0), 9);
  echoClient3.SetAttribute ("MaxPackets", UintegerValue (20));
  echoClient3.SetAttribute ("Interval", TimeValue (Seconds (5.)));
  echoClient3.SetAttribute ("PacketSize", UintegerValue (packetsize));
  ApplicationContainer clientApps3 = echoClient3.Install (wifiStaNodes3);
  clientApps3.Start (Seconds (1.0));
  clientApps3.Stop (Seconds (1000.0));

//  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  // Enable OLSR;
   OlsrHelper olsr;
   Ipv4StaticRoutingHelper staticRouting;

   Ipv4ListRoutingHelper list;
   list.Add (staticRouting, 0);
   list.Add (olsr, 10);
  Simulator::Stop (Seconds (900.0));

  ////////////////////////////////Animation
  AnimationInterface anim ("wireless-animation.xml"); // Mandatory
  for (uint32_t i = 0; i < mobileSinks.GetN (); ++i)
      {

	  char output[5] ;
	      std::sprintf(output,"MS %d",i);
      anim.UpdateNodeDescription (mobileSinks.Get (i), output); // Optional
      anim.UpdateNodeColor (mobileSinks.Get (i), 255, 255, 0); // Optional
      }
  for (uint32_t i = 0; i < wifiStaNodes.GetN (); ++i)
  {

  anim.UpdateNodeDescription (wifiStaNodes.Get (i), "STA"); // Optional
  anim.UpdateNodeColor (wifiStaNodes.Get (i), 255, 0, 0); // Optional
  }
  for (uint32_t i = 0; i < wifiStaNodes2.GetN (); ++i)
  {
  anim.UpdateNodeDescription (wifiStaNodes2.Get (i), "STA"); // Optional
  anim.UpdateNodeColor (wifiStaNodes2.Get (i), 255, 0, 0); // Optional
  }
  for (uint32_t i = 0; i < wifiStaNodes3.GetN (); ++i)
  {
  anim.UpdateNodeDescription (wifiStaNodes3.Get (i), "STA"); // Optional
  anim.UpdateNodeColor (wifiStaNodes3.Get (i), 255, 0, 0); // Optional
  }

  for (uint32_t i = 0; i < wifiApNodes.GetN (); ++i)
  {
	  char output[5] ;
	 	      std::sprintf(output,"AP %d",i);
  anim.UpdateNodeDescription (wifiApNodes.Get (i), output); // Optional
  anim.UpdateNodeColor (wifiApNodes.Get (i), 0, 255, 0); // Optional
  }

  for (uint32_t i = 0; i < uploadPoints.GetN (); ++i)
    {
	  char output[5] ;
	 	      std::sprintf(output,"UP %d",i);
    anim.UpdateNodeDescription (uploadPoints.Get (i), output); // Optional
    anim.UpdateNodeColor (uploadPoints.Get (i), 0, 255, 255); //int
    }
  anim.EnablePacketMetadata (); // Optional
  anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (500), Seconds (0.25)); //Optional
  //anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
  //anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional
  if (tracing == true)
   {
     phy.EnablePcap ("thirdfq", apDevices1.Get (0));
   }
 Simulator::ScheduleWithContext (0,Seconds (1.0), &showPosition,mobileSinks.Get(0),mobileSink[0],msInterfaces, upInterfaces,fwSinks);
 Simulator::ScheduleWithContext (1,Seconds (1.0), &showPosition,mobileSinks.Get(1),mobileSink[1],msInterfaces, upInterfaces,fwSinks);
 Simulator::ScheduleWithContext (2,Seconds (1.0), &showPosition,mobileSinks.Get(2),mobileSink[2],msInterfaces, upInterfaces,fwSinks);
 Simulator::ScheduleWithContext (3,Seconds (1.0), &showPosition,mobileSinks.Get(3),mobileSink[3],msInterfaces, upInterfaces,fwSinks);

 Simulator::Run ();
  Simulator::Destroy ();
  for(int k=0;k<4;k++){
  	 std::cout<<"\n Mobile Sink "<<k<< "'s current capacity : "<< size[k]<<std::endl;
  	std::cout<<" and has received packets "<< revPacket[k]<<std::endl;
  }
  for(int k=4;k<7;k++){
	  std::cout<<"\n RP "<<k-4<< "'s current capacity : "<< size[k]<<std::endl;
	    	std::cout<<"and has received packets "<< revPacket[k]<<std::endl;
    }
  for(int k=0;k<4;k++){
	 std::cout <<"\n Upload Point "<<k<<" received "<<uploadPacket[k] <<" packets ";
	 std::cout <<" with total size "<<uploadSize[k]<<std::endl;
     std::cout<< "\n Average data transmission delay: "<< averageDelay[k]<<std::endl;
  }
  return 0;
 }



