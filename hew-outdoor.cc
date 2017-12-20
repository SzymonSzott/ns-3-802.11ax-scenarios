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




#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>


// This is an implementation of the TGax (HEW) outdoor scenario.
using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hew-outdoor");

/*******  Forward declaration of functions *******/

int countAPs(int layers); // Count the number of APs per layer
double **calculateAPpositions(int h, int layers); // Calculate the positions of AP
void placeNodes(double **xy,NodeContainer &Nodes); // Place each node in 2D plane (X,Y)
double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations); //calculate positions of the stations
void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode);
void showPosition(NodeContainer &Nodes); // Show AP's positions (only in debug mode)
void PopulateARPcache ();

/*******  End of all forward declaration of functions *******/


double simulationTime = 10; //seconds
bool enableRtsCts = false; // RTS/CTS disabled by default
int stations = 1; //Stations per grid
int layers = 1; //Layers of hex grid
bool debug = false;
int h = 65; //distance between AP/2 (radius of hex grid)
std::string phy = "ac"; //802.11 PHY to use
int channelWidth = 20;
bool tracing = false;
/* Command line parameters */

int APs =  countAPs(layers);




int main (int argc, char *argv[])
{

	/* Command line parameters */

	CommandLine cmd;
	cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
	cmd.AddValue ("layers", "Number of layers in hex grid", layers);
	cmd.AddValue ("stations", "Number of stations in each grid", stations);
	cmd.AddValue ("debug", "Enable debug mode", debug);
	cmd.AddValue ("rts", "Enable RTS/CTS", enableRtsCts);
	cmd.AddValue ("phy", "Select PHY layer", phy);
	cmd.Parse (argc,argv);

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
		cout <<"Show AP's position: "<<endl;
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
		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ac);  //PHY standard
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("VhtMcs9"), "ControlMode", StringValue ("VhtMcs0"), "MaxSlrc", UintegerValue (10));
		wifiPhy.Set ("ShortGuardEnabled", BooleanValue (true));
		channelWidth = 80;
	}
	else if (phy == "ax"){
		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ax_5GHZ);
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("HeMcs11"),"ControlMode", StringValue ("HeMcs0"));
		wifiPhy.Set ("ShortGuardEnabled", BooleanValue (true));
		channelWidth = 80;
	}
	else if (phy == "n")
	{
		wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
		wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										"DataMode", StringValue ("HtMcs0"),
										"ControlMode", StringValue ("HtMcs0"));
		wifiPhy.Set ("ShortGuardEnabled", BooleanValue (1));
		channelWidth = 40;
	}
	else{
		std::cout<<"Given PHY doesn't exist or cannot be chosen. Choose one of the following:\n1. n\n2. ac\n3. ax"<<endl;
		exit(0);
	}

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
	wifiPhy.Set ("ShortGuardEnabled", BooleanValue (false));

	Ssid ssid = Ssid ("hew-outdoor-network");
	wifiMac.SetType ("ns3::ApWifiMac",
			"Ssid", SsidValue (ssid));

	NetDeviceContainer apDevices = wifiHelper.Install (wifiPhy, wifiMac, wifiApNodes);

	wifiPhy.Set ("TxPowerStart", DoubleValue (15.0));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0));
	wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
	wifiPhy.Set ("TxGain", DoubleValue (-2)); // for STA -2 dBi

	wifiMac.SetType ("ns3::StaWifiMac",
			"Ssid", SsidValue (ssid),
			"ActiveProbing", BooleanValue (false));

	NetDeviceContainer staDevices[APs];
	for(int i = 0; i < APs; ++i)
	{
		staDevices[i] = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNodes[i]);
	}

	/* Configure Internet stack */

	InternetStackHelper stack;
	stack.Install (wifiApNodes);
	for(int i = 0; i < APs; ++i)
	{
		stack.Install (wifiStaNodes[i]);
	}

	Ipv4AddressHelper address;
	address.SetBase ("10.1.0.0", "255.255.252.0");

	Ipv4InterfaceContainer StaInterfaces;
	Ipv4InterfaceContainer ApInterfaces;


	ApInterfaces = address.Assign (apDevices);

	for(int i = 0; i < APs; ++i)
	{
		StaInterfaces = address.Assign (staDevices[i]);
	}

	/* PopulateArpCache  */

	PopulateARPcache ();

	/* Configure applications */

        for(int i = 0; i < APs; ++i){
        for(int j = 0; j < stations; ++j)
	installTrafficGenerator(wifiStaNodes[i].Get(j),wifiApNodes.Get(i));
        }


	/* Configure tracing */

        if(tracing)
        {
	 wifiPhy.EnablePcap ("hew-outdoor", apDevices);
	 wifiPhy.EnablePcap ("hew-outdoor", staDevices[0]);
        }
	//EnablePcap ();

	/* Run simulation */

	Simulator::Stop (Seconds (simulationTime));
	Simulator::Run ();

	/* Calculate results */


	/* End of simulation */
	Simulator::Destroy ();
	return 0;
}

/***** Functions definition *****/

int countAPs(int layers){
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

void placeNodes(double **xy,NodeContainer &Nodes)
{
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

void showPosition(NodeContainer &Nodes)
{

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

double **calculateAPpositions(int h, int layers){

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

	srand(time(NULL));
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

void PopulateARPcache ()
{
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

void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode){

UdpServerHelper Server (9);
ApplicationContainer serverApps = Server.Install (toNode);
serverApps.Start (Seconds (0.0));
serverApps.Stop (Seconds (simulationTime + 1));

Ptr<Ipv4> ipv4 = toNode->GetObject<Ipv4> (); // Get Ipv4 instance of the node
Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal (); // Get Ipv4InterfaceAddress of xth interface.

UdpClientHelper Client (addr, 9);
Client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
Client.SetAttribute ("Interval", TimeValue (Seconds (1)));
Client.SetAttribute ("PacketSize", UintegerValue (1472));

ApplicationContainer clientApp;

clientApp= Client.Install (fromNode);
clientApp.Start (Seconds (1.0 + 0.05));
clientApp.Stop  (Seconds(simulationTime + 1));

}

/***** End of functions definition *****/
