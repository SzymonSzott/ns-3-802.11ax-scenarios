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
*/

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

#include<iostream>
#include<vector>
#include<math.h>
#include<string>
#include <fstream>

// This is an implementation of the TGax (HEW) outdoor scenario.

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("hew-outdoor");

int countAPs(int layers); // Count the number of APs per layer
double **calculate_AP_positions(int h, int layers); //Calculate the positions of AP
void placeAP(double APcoordinates[][19],NodeContainer &accessPoint);//  Set each AP in 2D plane (X,Y)
void showPosition(NodeContainer &accessPoint); // show AP's positions if it runs in debug mode


int main (int argc, char *argv[])
{

	/* Initialize parameters */
	double simulationTime = 10; //seconds
  int layers=3;
  bool debug=true;
	int h=65; //distance between AP/2
  int rows = 2; // row for x,y coordinates
  int columns = 19; // columns for the 19 APs
  double APcoordinates[rows][columns];
	/* Command line parameters */

	CommandLine cmd;
	cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
  cmd.AddValue ("layers", "Number of layers in hex grid", layers);
  cmd.AddValue ("debug", "Enable debug mode", debug);
	cmd.Parse (argc,argv);

	/* Position APs */

    if(debug){
        std::cout << "There are "<< countAPs(layers) << " APs in " << layers << " layers.\n";
    }



// calculate_AP_position function
	double ** APpositions;
	APpositions = calculate_AP_positions(h, layers);

          // getting the coordinates from 2D array
        for (int i = 0; i < rows; i++)
           {
             for (int j = 0; j < columns; j++)
              {
                APcoordinates[i][j] =  APpositions[i][j];
              }
           }
	//how it works
	if(debug){
		for (int m=0; m<countAPs(layers);m++){
			std::cout << APpositions[0][m]<< "\t" <<APpositions[1][m]<<std::endl;
		}
	}

        NodeContainer wifiApNodes ;
        wifiApNodes.Create(countAPs(layers)); // create APnode according to the number of them
        placeAP(APcoordinates,wifiApNodes);

        if(debug)
           {
              showPosition(wifiApNodes);
           }
       /* POSITION STA */

	//foreach (AP) {placeSTA(Xap, Yap, nSta, radius (=ICD/2))}

	/* Configure propagation model */

	/* Configure MAC and PHY */

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

int countAPs(int layers){
    int APsum=1; //if 1 layer then 1 AP
    if(layers>1) {
        for(int i=0; i<layers; i++) {
              APsum=APsum+6*i;
        }
    }
    return APsum;
}

void placeAP(double APcoordinates[][19],NodeContainer &accessPoint)
{

    const int columns = 19;// there is no sense to declare the number of rows

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();


              for(int i = 0; i < columns; ++i)
               {
                 positionAlloc->Add (Vector (APcoordinates[0][i],APcoordinates[1][i], 0.0));
               }

    mobility.SetPositionAllocator (positionAlloc);

    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (accessPoint);

}

void showPosition(NodeContainer &accessPoint)
{

     int APnumber = 1;
    for(NodeContainer::Iterator nAP = accessPoint.Begin (); nAP !=accessPoint .End (); ++nAP)
    {
        Ptr<Node> object = *nAP;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
        NS_ASSERT (position != 0);
        Vector pos = position->GetPosition ();
        std::cout <<"AP"<<"("<<APnumber<<")"<<" has coordinates "<< "(" << pos.x << ", " << pos.y <<")" << std::endl;
        ++APnumber;

    }

}


double **calculate_AP_positions(int h, int layers){

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


	//std::string file_name="coord.ods";
	//std::ofstream coordfile;
	//coordfile.open (file_name.c_str(), std::ios::app);



double** AP_co=0;
	AP_co = new double*[2];
	AP_co[0]=new double[APnum];
	AP_co[1]=new double[APnum];

	for (int p=0;p<APnum;p++){
			AP_co[0][p]=x_co[p];
			AP_co[1][p]=y_co[p];
	}


	//	for ( int m = 0; m < APnum; ++m){
	//	coordfile<<x_co[m]<<"\t"<<y_co[m]<<std::endl;
	//	}

	//coordfile.close();
	return AP_co;
	}
