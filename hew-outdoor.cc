/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors:
 * Szymon Szott <szott@kt.agh.edu.pl>
 * Joanna Czepiec <joanna.czepiec7@gmail.com>
 * Geovani Teca <tecageovani@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-address.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>

// This is an implementation of the TGax (HEW) outdoor scenario.
using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hew-outdoor");

/*******  Forward declaration of functions *******/

int countAPs(int layers); // Count the number of APs per layer
double **calculateAPpositions(int h, int layers); // Calculate the positions of AP
void placeNodes(double **xy,NodeContainer &Nodes); // Place each node in 2D plane (X,Y)
double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations); //calculate positions of the stations
void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, std::string offeredLoad, int packetSize, int simulationTime, int warmupTime);
void showPosition(NodeContainer &Nodes); // Show AP's positions (only in debug mode)
void PopulateARPcache ();
void ftpApplicationSetup (Ptr<Node> client, Ptr<Node> server, int port, double start, double stop); //FTP traffic generator

/*******  End of all forward declaration of functions *******/

int main (int argc, char *argv[])
{
	/* Variable declarations */

	bool enableRtsCts = false; // RTS/CTS disabled by default
	int stations = 5; //Stations per grid
	int layers = 1; //Layers of hex grid
	bool debug = false;
	int h = 30; //distance between AP/2 (radius of hex grid)
	string phy = "ac"; //802.11 PHY to use
	int channelWidth = 20;
	bool pcap = false;
	//string offeredLoad = "1"; //Mbps
	bool highMcs = false; //Use of high MCS settings
	string mcs;
  std::string offeredLoad = "1"; //Mbps
	int simulationTime = 10;
	int warmupTime = 1;
	int packetSize = 1472;
    int nFtp = 0;
	/* Command line parameters */

	CommandLine cmd;
	cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
	cmd.AddValue ("layers", "Number of layers in hex grid", layers);
	cmd.AddValue ("stations", "Number of stations in each grid", stations);
	cmd.AddValue ("debug", "Enable debug mode", debug);
	cmd.AddValue ("rts", "Enable RTS/CTS", enableRtsCts);
	cmd.AddValue ("phy", "Select PHY layer", phy);
	cmd.AddValue ("highMcs", "Select high or low MCS settings", highMcs);
	cmd.AddValue ("pcap", "Enable PCAP generation", pcap);
	cmd.AddValue ("offeredLoad", "Offered Load [Mbps]", offeredLoad);
	cmd.AddValue ("packetSize", "Packet size [s]", packetSize);
	cmd.AddValue ("warmupTime", "Warm-up time [s]", warmupTime);
    cmd.AddValue ("nFtp", "Number of stations transmitting ftp traffic", nFtp);
	cmd.Parse (argc,argv);

	int APs =  countAPs(layers);

    if (stations < nFtp){
        throw std::invalid_argument("Number of stations transmitting FTP traffic cannot be higher than number of all stations");
    }

	if(debug) {
		LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
		LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);
	}

	/* Enable or disable RTS/CTS */

	if (enableRtsCts) {
		Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("100"));
	}
	else {
		Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("1100000"));
	}

	/* Calculate the number of  APs */

	if(debug){
		std::cout << "There are "<< APs << " APs in " << layers << " layers.\n";
	}

	/* Calculate AP positions */

	double ** APpositions;
	APpositions = calculateAPpositions(h,layers);

	NodeContainer wifiApNodes ;
	wifiApNodes.Create(APs);

	/* Place each AP in 3D (X,Y,Z) plane */

	placeNodes(APpositions,wifiApNodes);

	/* Display AP positions */

	if(debug)
	{
		cout << "Show AP's position: "<< endl;
		showPosition(wifiApNodes);
	}

	/* Place each station randomly around its AP */

	NodeContainer wifiStaNodes[APs];
	for(int APindex = 0; APindex < APs; ++APindex)
	{
		wifiStaNodes[APindex].Create(stations);
		double **STApositions;
		STApositions = calculateSTApositions(APpositions[0][APindex], APpositions[1][APindex], h, stations);

		/* Place each stations in 3D (X,Y,Z) plane */

		placeNodes(STApositions,wifiStaNodes[APindex]);

		/* Display STA positions */

		if(debug)
		{
			cout <<"Show Stations around AP("<<APindex<<"):"<<endl;
			showPosition(wifiStaNodes[APindex]);
		}
	}

	/* Configure propagation model */

	WifiMacHelper wifiMac;
	WifiHelper wifiHelper;
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

	if (phy == "ac"){
		if(highMcs == 1)
		{
			mcs ="VhtMcs9";
		}
		else
		{
			mcs ="VhtMcs0";
		}
		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ac);
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (mcs), "ControlMode", StringValue (mcs), "MaxSlrc", UintegerValue (10));

		channelWidth = 80;
	}
	else if (phy == "ax"){
		if(highMcs == 1)
		{
			mcs ="HeMcs11";
		}
		else
		{
			mcs ="HeMcs0";
		}

		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ax_5GHZ);
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (mcs),"ControlMode", StringValue (mcs));

		channelWidth = 80;
	}
	else if (phy == "n")
	{
		if(highMcs == 1)
		{
			mcs ="HtMcs7";
		}
		else
		{
			mcs ="HtMcs0";
		}
		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
				"DataMode", StringValue (mcs),
				"ControlMode", StringValue (mcs));

		channelWidth = 40;
	}
	else {
		std::cout<<"Given PHY doesn't exist or cannot be chosen. Choose one of the following:\n1. n\n2. ac\n3. ax"<<endl;
		exit(0);
	}
	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue (NanoSeconds (800))); // LONG GI set

	/* Set up Channel */

	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	//wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel");

	/* Set channel width */
	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));


	/* Configure MAC and PHY */


	wifiPhy.SetChannel (wifiChannel.Create ());
	wifiPhy.Set ("TxPowerStart", DoubleValue (20.0));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (20.0));
	wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
	wifiPhy.Set ("TxGain", DoubleValue (0));
	wifiPhy.Set ("RxGain", DoubleValue (0));
	wifiPhy.Set ("RxNoiseFigure", DoubleValue (7));
	/*	wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-79));
		wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3)); */
	wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardEnabled", BooleanValue (false));

	NetDeviceContainer apDevices;
	Ssid ssid;

	for(int i = 0; i < APs; ++i) {
		ssid = Ssid ("hew-outdoor-network-" + std::to_string(i));
		wifiMac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));
		NetDeviceContainer apDevice = wifiHelper.Install (wifiPhy, wifiMac, wifiApNodes.Get(i));
		apDevices.Add(apDevice);
	}

	wifiPhy.Set ("TxPowerStart", DoubleValue (15.0));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0));
	wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
	wifiPhy.Set ("TxGain", DoubleValue (-2)); // for STA -2 dBi

	NetDeviceContainer staDevices[APs];

	for(int i = 0; i < APs; ++i) {
		ssid = Ssid ("hew-outdoor-network-" + std::to_string(i));
		wifiMac.SetType ("ns3::StaWifiMac",	"Ssid", SsidValue (ssid),"ActiveProbing", BooleanValue (false));
		NetDeviceContainer staDevice = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNodes[i]);
		staDevices[i].Add(staDevice);
	}

	/* Configure Internet stack */

	InternetStackHelper stack;
	stack.Install (wifiApNodes);
	for(int i = 0; i < APs; ++i)
	{
		stack.Install (wifiStaNodes[i]);
	}

	// Ipv4AddressHelper address;
	// address.SetBase ("10.1.0.0", "255.255.252.0");
	//
	// Ipv4InterfaceContainer StaInterfaces;
	// Ipv4InterfaceContainer ApInterfaces;
	//
	// ApInterfaces = address.Assign (apDevices);
	//
	// for(int i = 0; i < APs; ++i)
	// {
	//
	// 	StaInterfaces = address.Assign (staDevices[i]);
	// }



	Ipv4AddressHelper address;

	for(int i = 0; i < APs; ++i)
	{
		std::string addrString;
		addrString =  "10.1." + to_string(i) + ".0";
		const char *cstr = addrString.c_str(); //convert to constant char
		address.SetBase (Ipv4Address(cstr), "255.255.255.0");
		address.Assign (apDevices.Get(i));
		address.Assign (staDevices[i]);
	}

	/* PopulateArpCache  */

	PopulateARPcache ();

	/* Configure applications */

	int port=9;
	for(int i = 0; i < APs; ++i){
		for(int j = 0; j < nFtp; ++j)
			//installTrafficGenerator(wifiStaNodes[i].Get(j),wifiApNodes.Get(i), port++, offeredLoad, packetSize, simulationTime, warmupTime);
            ftpApplicationSetup(wifiStaNodes[i].Get(j),wifiApNodes.Get(i), port++, warmupTime, simulationTime);
        for(int j = nFtp; j < stations; ++j)
            installTrafficGenerator(wifiStaNodes[i].Get(j),wifiApNodes.Get(i), port++, offeredLoad, packetSize, simulationTime, warmupTime);
	}


	/* Configure tracing */

	//EnablePcap ();

	if(pcap) {
        wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
		wifiPhy.EnablePcap ("hew-outdoor", apDevices);
		for(int i = 0; i < APs; ++i){
			wifiPhy.EnablePcap ("hew-outdoor", staDevices[i]);
		}
	}

	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

	/* Run simulation */

	Simulator::Stop(Seconds(simulationTime));
	Simulator::Run ();

	/* Calculate results */
	double flowThr;
	double flowDel;

	ofstream myfile;
	myfile.open ("hew-outdoor.csv", ios::app);


	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
		auto time = std::time(nullptr); //Get timestamp
		auto tm = *std::localtime(&time);
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
		flowThr=i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1024 / 1024;
		flowDel=i->second.delaySum.GetSeconds () / i->second.rxPackets;
		NS_LOG_UNCOND ("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\tThroughput: " <<  flowThr  << " Mbps\tTime: " << i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds () << "\t Delay: " << flowDel << " s \n");
		myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << offeredLoad << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << t.destinationAddress << "," << flowThr << "," << flowDel;
		myfile << std::endl;
	}
	myfile.close();

	/* End of simulation */
	Simulator::Destroy ();
	return 0;
}

/***** Functions definition *****/

int countAPs(int layers) {
	int APsum=1; //if 1 layer then 1 AP
	if(layers>1)
	{
		for(int i=0; i<layers; i++)
		{
			APsum=APsum+6*i;
		}
	}

	return APsum;
}

void placeNodes(double **xy,NodeContainer &Nodes) {
	uint32_t nNodes = Nodes.GetN ();
	double height = 0.0;
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	if(xy[0][0] == 0 && xy[1][0] == 0)
		height = 10.0;
	else
		height = 1.5;

	for(uint32_t i = 0; i < nNodes; ++i)
	{
		positionAlloc->Add (Vector (xy[0][i],xy[1][i],height));
	}

	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (Nodes);
}

void showPosition(NodeContainer &Nodes) {

	uint32_t NodeNumber = 0;

	for(NodeContainer::Iterator nAP = Nodes.Begin (); nAP != Nodes.End (); ++nAP)
	{
		Ptr<Node> object = *nAP;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		NS_ASSERT (position != 0);
		Vector pos = position->GetPosition ();
		std::cout <<"Node Number"<<"("<< NodeNumber <<")"<<" has coordinates "<< "(" << pos.x << ", "<< pos.y <<", "<< pos.z <<")" << std::endl;
		++NodeNumber;
	}
}

double **calculateAPpositions(int h, int layers) {

	float sq=sqrt(3);
	float first_x=0; // coordinates of the first AP
	float first_y=0;
	float x=0;
	float y=0;


	int APnum=countAPs(layers); //number of AP calculated by the AP_number function

	std::vector <float> x_co;
	std::vector <float> y_co;

	x_co.push_back(first_x);
	y_co.push_back(first_y);

	for(int lay=1; lay<layers;lay++){
		for(int k=1;k<=7;k++){
			if(k==1){
				x=x+h*sq;
				y=y+h;

				x_co.push_back(x);
				y_co.push_back(y);

			}
			else if(k==2){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){

					x=x-h*sq;
					y=y+h;

					x_co.push_back(x);
					y_co.push_back(y);
				}

			}

			else if(k==3){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){

					x=x-h*sq;
					y=y-h;

					x_co.push_back(x);
					y_co.push_back(y);
				}
			}
			else if(k==4){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){

					y=y-2*h;

					x_co.push_back(x);
					y_co.push_back(y);
				}
			}
			else if(k==5){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){

					x=x+h*sq;
					y=y-h;

					x_co.push_back(x);
					y_co.push_back(y);
				}
			}
			else if(k==6){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){

					x=x+h*sq;
					y=y+h;

					x_co.push_back(x);
					y_co.push_back(y);
				}
			}
			else if(k==7){
				for(int hex_lay=1; hex_lay<=lay; hex_lay++){
					y=y+2*h;
					if(hex_lay<lay){
						x_co.push_back(x);
						y_co.push_back(y);
					}
				}
			}
		}
	}



	double** AP_co=0;
	AP_co = new double*[2];
	AP_co[0]=new double[APnum];
	AP_co[1]=new double[APnum];

	for (int p=0;p<APnum;p++){
		AP_co[0][p]=x_co[p];
		AP_co[1][p]=y_co[p];
	}

	return AP_co;
}

double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations) {

	double PI  =3.141592653589793238463;


	double tab[2][n_stations];
	double** sta_co=0;
	sta_co = new double*[2];
	sta_co[0]=new double[n_stations];
	sta_co[1]=new double[n_stations];
	double ANG = 2*PI;

	float X=1;
	for(int i=0; i<n_stations; i++){
		float sta_x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/X));
		tab[0][i]= sta_x*h;

	}

	for (int j=0; j<n_stations; j++){
		float angle = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/ANG));
		tab[1][j]=angle;

	}
	for ( int k=0; k<n_stations; k++){
		sta_co[0][k]=x_ap+cos(tab[1][k])*tab[0][k];
		sta_co[1][k]=y_ap+sin(tab[1][k])*tab[0][k];

	}


	return sta_co;
}

void PopulateARPcache () {
	Ptr<ArpCache> arp = CreateObject<ArpCache> ();
	arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );

	for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
	{
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip !=0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

		for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++)
		{
			Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
			NS_ASSERT (ipIface != 0);
			Ptr<NetDevice> device = ipIface->GetDevice ();
			NS_ASSERT (device != 0);
			Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );

			for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++)
			{
				Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
				if (ipAddr == Ipv4Address::GetLoopback ())
					continue;

				ArpCache::Entry *entry = arp->Add (ipAddr);
				Ipv4Header ipv4Hdr;
				ipv4Hdr.SetDestination (ipAddr);
				Ptr<Packet> p = Create<Packet> (100);
				entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr));
				entry->MarkAlive (addr);
			}
		}
	}

	for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
	{
		Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip !=0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

		for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
		{
			Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
			ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
		}
	}
}

void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, std::string offeredLoad, int packetSize, int simulationTime, int warmupTime ) {

	Ptr<Ipv4> ipv4 = toNode->GetObject<Ipv4> (); // Get Ipv4 instance of the node
	Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.

	ApplicationContainer sourceApplications, sinkApplications;

	uint8_t tosValue = 0x70; //AC_BE
	
	//Add random fuzz to app start time
	double min = 0.0;
	double max = 1.0;
	Ptr<UniformRandomVariable> fuzz = CreateObject<UniformRandomVariable> ();
	fuzz->SetAttribute ("Min", DoubleValue (min));
	fuzz->SetAttribute ("Max", DoubleValue (max));		

	InetSocketAddress sinkSocket (addr, port);
	sinkSocket.SetTos (tosValue);
	OnOffHelper onOffHelper ("ns3::UdpSocketFactory", sinkSocket);
	onOffHelper.SetConstantRate (DataRate (offeredLoad + "Mbps"), packetSize);
	sourceApplications.Add (onOffHelper.Install (fromNode)); //fromNode
	PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
	sinkApplications.Add (packetSinkHelper.Install (toNode)); //toNode

	sinkApplications.Start (Seconds (warmupTime));
	sinkApplications.Stop (Seconds (simulationTime));
	sourceApplications.Start (Seconds (warmupTime+fuzz->GetValue ()));
	sourceApplications.Stop (Seconds (simulationTime));




}

void ftpApplicationSetup (Ptr<Node> client, Ptr<Node> server, int port, double start, double stop)
{
  Ptr<Ipv4> ipv4Server = server->GetObject<Ipv4> ();

  uint8_t tosValue = 0x28; //AC_BK

  Ipv4InterfaceAddress iaddrServer = ipv4Server->GetAddress (1,0);
  Ipv4Address ipv4AddrServer = iaddrServer.GetLocal ();
  //int port = 5000;
  
  ApplicationContainer sinkApplications, sourceApplications;
  InetSocketAddress sinkSocket (ipv4AddrServer, port);
  sinkSocket.SetTos (tosValue);
  // Equipping the source  node with OnOff Application used for sending
  OnOffHelper onoff ("ns3::UdpSocketFactory", sinkSocket ); //only for UL traffic
  onoff.SetAttribute ("PacketSize", UintegerValue (500));
  //onoff.SetAttribute ("Remote", AddressValue (InetSocketAddress (ipv4AddrServer, port)));
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]")); 
  onoff.SetAttribute ("OffTime", StringValue ("ns3::ExponentialRandomVariable[Mean=0.5|Bound=10]")); // here the DataRate can be adjusted
  sourceApplications.Add (onoff.Install(client));
  PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket);
  sinkApplications.Add (packetSinkHelper.Install (server)); //toNode
  
  
  sinkApplications.Start(Seconds(start));
  sinkApplications.Stop(Seconds(stop));
  sourceApplications.Start (Seconds (start));
  sourceApplications.Stop (Seconds (stop));
}
/***** End of functions definition *****/
