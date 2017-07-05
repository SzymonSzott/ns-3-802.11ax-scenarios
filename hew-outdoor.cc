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

#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>


// This is an implementation of the TGax (HEW) outdoor scenario.

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hew-outdoor");

/*******  Foward declaration of functions *******/

int countAPs(int layers); // Count the number of APs per layer
double **calculateAPpositions(int h, int layers); // Calculate the positions of AP
void placeNodes(double **xy,NodeContainer &Nodes); // Place each node in 2D plane (X,Y)
double **calculateSTApositions(double x_ap, double y_ap, int h, int n_stations); //calculate positions of the stations
void showPosition(NodeContainer &Nodes); // Show AP's positions (only in debug mode)


/*******  End of all foward declaration of functions *******/

int main (int argc, char *argv[])
{

	/* Initialize parameters */

	double simulationTime = 10; //seconds
	bool enableRtsCts = false; // RTS/CTS disabled by default
	int stations = 50;
	int layers = 3;
	bool debug = false;
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

	if(enableRtsCts)
	{
		UintegerValue ctsThr = (enableRtsCts ? UintegerValue (100) : UintegerValue (65535));
		Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
	}

	/* Position APs */
	if(debug){
		std::cout << "There are "<< APs << " APs in " << layers << " layers.\n";
	}

	/* calculate_AP_position function */

	double ** APpositions;
	APpositions = calculateAPpositions(h,layers);


	/* Only for TEST purpose */

	if(debug)
	{
		for (int m = 0; m < APs; m++)
		{
			std::cout << APpositions[0][m]<< "\t" <<APpositions[1][m]<<std::endl;
		}
	}

	NodeContainer wifiApNodes ;
	wifiApNodes.Create(APs);

	/* Place each AP in 2D (X,Y) plane */

	placeNodes(APpositions,wifiApNodes);

	/* Only for TEST purpose */

	if(debug)
	{
		showPosition(wifiApNodes);
	}

	/* POSITION STA */

	NodeContainer wifiStaNodes ;
	wifiApNodes.Create(stations);

	double ** STApositions;

	for (int i=0; i<APs; i++){
		STApositions = calculateSTApositions(APpositions[0][i], APpositions[1][i], h, stations);

		/* Place each stations in 2D (X,Y) plane */

		placeNodes(STApositions,wifiStaNodes);

		if(debug){
			std::cout<< "AP number: "<<i<<std::endl;
			for (int j=0; j<stations; j++){
				std::cout<< STApositions[0][j] << "\t" << STApositions[1][j] <<std::endl;
			}
		}
	}


	/* Configure propagation model */

	WifiMacHelper wifiMac;
	WifiHelper wifiHelper;
	wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211ac);  //PHY standard


	/* Set up Channel */
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel", "Frequency", DoubleValue (5e9));

	Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (80)); //set channel width

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
	wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("VhtMcs9"), "ControlMode", StringValue ("VhtMcs0"), "MaxSlrc", UintegerValue (10));
	wifiPhy.Set ("ShortGuardEnabled", BooleanValue (false));

	NetDeviceContainer apDevices = wifiHelper.Install (wifiPhy, wifiMac, wifiApNodes);

	wifiPhy.Set ("TxPowerStart", DoubleValue (15.0)); 
	wifiPhy.Set ("TxPowerEnd", DoubleValue (15.0)); 
	wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
	wifiPhy.Set ("TxGain", DoubleValue (-2)); // for STA -2 dBi
	NetDeviceContainer staDevices = wifiHelper.Install (wifiPhy, wifiMac, wifiStaNodes);

	//PopulateArpCache ();

	/* Configure Internet stack */

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
	if(layers>1) {
		for(int i=0; i<layers; i++) {
			APsum=APsum+6*i;
		}
	}
	return APsum;
}


void placeNodes(double **xy,NodeContainer &Nodes)
{

	uint32_t nNodes = Nodes.GetN ();

	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	for(uint32_t i = 0; i < nNodes; ++i)
	{
		positionAlloc->Add (Vector (xy[0][i],xy[1][i], 0.0));
	}

	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (Nodes);
}

void showPosition(NodeContainer &Nodes)
{

	int APnumber = 1;
	for(NodeContainer::Iterator nAP = Nodes.Begin (); nAP != Nodes.End (); ++nAP)
	{
		Ptr<Node> object = *nAP;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
		NS_ASSERT (position != 0);
		Vector pos = position->GetPosition ();
		std::cout <<"AP"<<"("<<APnumber<<")"<<" has coordinates "<< "(" << pos.x << ", " << pos.y <<")" << std::endl;
		++APnumber;

	}

}

double **calculateAPpositions(int h, int layers){

	float sq=sqrt(3);
	float first_x=0; // coordinates of the first AP
	float first_y=0;
	float x=0;
	float y=0;


	int APnum=countAPs(layers); //number of AP calculated by the AP_number function

	std::cout<<APnum<<std::endl;


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
/***** End of functions definition *****/
