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
 * Authors: Szymon Szott <szott@kt.agh.edu.pl>
   Geovani Teca <tecageovani@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-l3-protocol.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>


// This is an implementation of the TGax (HEW) outdoor scenario.
using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hew-outdoor");

/*******  Foward declaration of functions *******/

int countAPs(int layers); // Count the number of APs per layer
double **calculateAPpositions(int h, int layers); // Calculate the positions of AP
void placeNodes(double **xy,NodeContainer &Nodes); // Place each node in 2D plane (X,Y)
double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations); //calculate positions of the stations
void showPosition(NodeContainer &Nodes); // Show AP's positions (only in debug mode)
void PopulateARPcache ();

/*******  End of all foward declaration of functions *******/

int main (int argc, char *argv[])
{

	/* Initialize parameters */

	double simulationTime = 10; //seconds
	bool enableRtsCts = false; // RTS/CTS disabled by default
	int stations = 50;
	int layers = 3;
	bool debug = true;
	int h = 65; //distance between AP/2 (radius of hex grid)
	int APs =  countAPs(layers);

	/* Command line parameters */

	CommandLine cmd;
	cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
	cmd.AddValue ("layers", "Number of layers in hex grid", layers);
	cmd.AddValue ("debug", "Enable debug mode", debug);
	cmd.AddValue ("rts", "Enable RTS/CTS", enableRtsCts);
	cmd.Parse (argc,argv);

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

	/* calculate_AP_position function */

	double ** APpositions;
	APpositions = calculateAPpositions(h,layers);

	NodeContainer wifiApNodes ;
	wifiApNodes.Create(APs);

	/* Place each AP in 3D (X,Y,Z) plane */

	placeNodes(APpositions,wifiApNodes);

	/* Only for TEST purpose */

	if(debug)
	{
           cout <<"Show AP's position : "<<endl;
	   showPosition(wifiApNodes);
	}

	/* POSITION STA */

	NodeContainer wifiStaNodes[APs];
	for(int APindex = 0; APindex < APs; ++APindex)
	  {
	      wifiStaNodes[APindex].Create(stations);
	      double **STApositions;
              STApositions = calculateSTApositions(APpositions[0][APindex], APpositions[1][APindex], h, stations);

	  /* Place each stations in 3D (X,Y,Z) plane */

	     placeNodes(STApositions,wifiStaNodes[APindex]);

          /* Only for TEST purpose */

             if(debug)
                {
                 cout <<"Show Stations around AP("<<APindex<<"):"<<endl;
                 showPosition(wifiStaNodes[APindex]);
                }
	  }

 	/* Configure propagation model */

	WifiMacHelper wifiMac;
	WifiHelper wifiHelper;
	wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ac);  //PHY standard
        wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("VhtMcs9"), "ControlMode", StringValue ("VhtMcs0"), "MaxSlrc", UintegerValue (10));

	/* Set up Channel */

	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel", "Frequency", DoubleValue (5e9));



	/* Configure MAC and PHY */

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy.SetChannel (wifiChannel.Create ());
	wifiPhy.Set ("TxPowerStart", DoubleValue (20.0));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (20.0));
	wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
	wifiPhy.Set ("TxGain", DoubleValue (0));
	wifiPhy.Set ("RxGain", DoubleValue (0));
	wifiPhy.Set ("RxNoiseFigure", DoubleValue (7));
	wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-79));
	wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
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

        Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (80)); //set channel width

	/* Configure Internet stack */

	InternetStackHelper stack;
	stack.Install (wifiApNodes);
	for(int i = 0; i < APs; ++i)
	   {
             stack.Install (wifiStaNodes[i]);
           }

	Ipv4AddressHelper address;
	address.SetBase ("10.1.0.0", "255.255.252.0");
	address.Assign (apDevices);
        for(int i = 0; i < APs; ++i)
           {
             address.Assign (staDevices[i]);
           }

	/* PopulateArpCache  */

	PopulateARPcache ();

	/* Configure applications */

	/* Configure tracing */

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

        uint32_t NodeNumber = 1;

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

//Populate ARP cache

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



/***** End of functions definition *****/
