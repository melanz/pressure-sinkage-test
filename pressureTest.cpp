///////////////////////////////////////////////////
//      
//   Demo code about   
//   
//     - collisions and contacts 
//
//       (This is just a possible method of integration
//       of Chrono::Engine + Irrlicht: many others 
//       are possible.)
//     
//	 CHRONO   
//   ------
//   Multibody dinamics engine 
//   
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
    
  
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpIterativeSORmultithread.h"
#include "lcp/ChLcpIterativeSolver.h"

#include "physics/ChMaterialSurface.h"
#include "irrlicht_interface/ChIrrApp.h"

#include "core/ChFileutils.h"

#include <irrlicht.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
 
// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

using namespace std;

vector<char*> bodyTypes;
vector<double> particleRadii;
ChVector<double> forceOnGround;
double cielingHeight;

// Define the callback class for reporting all contacts via callback proxy
class _reporter_class : public chrono::ChReportContactCallback 
{
public:
	virtual bool ReportContactCallback (
					const ChVector<>& pA,				
					const ChVector<>& pB,				
					const ChMatrix33<>& plane_coord,	
					const double& distance,				
					const float& mfriction,			  	
					const ChVector<>& react_forces,		
					const ChVector<>& react_torques,	
					collision::ChCollisionModel* modA,	
					collision::ChCollisionModel* modB) 
	{	
		ChMatrix33<> localmatr(plane_coord);
		ChVector<> n1 = localmatr.Get_A_Xaxis();

		ChVector<> absreac = localmatr * react_forces;	//comment out localmatr to get relative forces!

		//ChBody* bodyA = (*msystem->Get_bodylist())[modA->GetPhysicsItem()->GetIdentifier()];
		//ChBody* bodyB = (*msystem->Get_bodylist())[modB->GetPhysicsItem()->GetIdentifier()];

		if((modA->GetPhysicsItem()->GetIdentifier()==bodyToGetForceOn->GetIdentifier()||modB->GetPhysicsItem()->GetIdentifier()==bodyToGetForceOn->GetIdentifier())&&(absreac.Length()))
		{
			//printf("%.3f %.3f %.3f\n", absreac.x, absreac.y, absreac.z);
			forceOnGround.x+=absreac.x;
			forceOnGround.y+=absreac.y;
			forceOnGround.z+=absreac.z;
		}
		return true; // to continue scanning contacts
	}
	// Data 
	ChSharedPtr<ChBody> bodyToGetForceOn;
	ChSystem* msystem;
};

struct bodyLocation {
	double x;
	double y;
	double z;
	double e0;
	double e1;
	double e2;
	double e3;
};

int importBodyLocations(char* fileName, vector<bodyLocation>* bodyLocations)
{
	double pos_x=0;
	double pos_y=0;
	double pos_z=0;
	double rot_0=0;
	double rot_1=0;
	double rot_2=0;
	double rot_3=0;
	double vel_x=0;
	double vel_y=0;
	double vel_z=0;
	int bodyType = 0;
	double radius = 0;

	std::string temp_data;
	ifstream ifile(fileName);

	bodyLocation bodyLoc;
	while(getline(ifile,temp_data))//for (int i = 0; i < numBodies; i++)
	{ 
		//getline(ifile,temp_data);
		for(int i=0; i<temp_data.size(); ++i){
			if(temp_data[i]==','){temp_data[i]=' ';}
		}

		std::stringstream ss(temp_data);
		ss>>pos_x>>pos_y>>pos_z>>rot_0>>rot_1>>rot_2>>rot_3>>vel_x>>vel_y>>vel_z>>bodyType>>radius;

		bodyLoc.x = pos_x;
		bodyLoc.y = pos_y;
		bodyLoc.z = pos_z;
		bodyLoc.e0 = rot_0;
		bodyLoc.e1 = rot_1;
		bodyLoc.e2 = rot_2;
		bodyLoc.e3 = rot_3;
		

		if(bodyType==0)
		{
			bodyLocations->push_back(bodyLoc);
			particleRadii.push_back(radius);
		}
		if(bodyType==4) cielingHeight = pos_y;
	}

	return 0;
}

double getRandomNumber(double min, double max)
{
   // x is in [0,1[
   double x = rand()/static_cast<double>(RAND_MAX);

   // [0,1[ * (max - min) + min is in [min,max[
   double that = min + ( x * (max - min) );

   return that;
}

int generateRockObject(std::string filename, double rockRadius, double vertPertConstant)
{
	//char filename[100];
	//sprintf(filename, "./data/directShear/rocks/rock%d.obj", rockIndex);
	ChStreamOutAsciiFile rockFile(filename.c_str());

	rockFile << "# OBJ file created by ply_to_obj.c\n#";//\ng Object001";

	// add vertices
	//1
	ChVector<double> vertex = ChVector<>(-0.57735,  -0.57735,  0.57735);
	ChVector<double> vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " <<vertex.z;
	//2
	vertex = ChVector<>(0.934172,  0.356822,  0);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//3
	vertex = ChVector<>(0.934172,  -0.356822,  0);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//4
	vertex = ChVector<>(-0.934172,  0.356822,  0);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//5
	vertex = ChVector<>(-0.934172,  -0.356822,  0);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//6
	vertex = ChVector<>(0,  0.934172,  0.356822);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//7
	vertex = ChVector<>(0,  0.934172,  -0.356822);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//8
	vertex = ChVector<>(0.356822,  0,  -0.934172);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//9
	vertex = ChVector<>(-0.356822,  0,  -0.934172);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//10
	vertex = ChVector<>(0,  -0.934172,  -0.356822);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//11
	vertex = ChVector<>(0,  -0.934172,  0.356822);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//12
	vertex = ChVector<>(0.356822,  0,  0.934172);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//13
	vertex = ChVector<>(-0.356822,  0,  0.934172);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//14
	vertex = ChVector<>(0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//15
	vertex = ChVector<>(0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//16
	vertex = ChVector<>(-0.57735,  0.57735,  -0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//17
	vertex = ChVector<>(-0.57735,  0.57735,  0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//18
	vertex = ChVector<>(0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//19
	vertex = ChVector<>(0.57735,  -0.57735,  0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;
	//20
	vertex = ChVector<>(-0.57735,  -0.57735,  -0.57735);
	vertPert = ChVector<>(vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1),vertPertConstant*getRandomNumber(-1,1));
	vertex = rockRadius*(vertex+vertPert);
	rockFile << "\nv " << vertex.x << " " << vertex.y << " " << vertex.z;

	// add faces
	rockFile << "\n\nf  19  3  2\nf  12  19  2\nf  15  12  2\nf  8  14  2\nf  18  8  2\nf  3  18  2\nf  20  5  4\nf  9  20  4\nf  16  9  4\nf  13  17  4\nf  1  13  4\nf  5  1  4\nf  7  16  4\nf  6  7  4\nf  17  6  4\nf  6  15  2\nf  7  6  2\nf  14  7  2\nf  10  18  3\nf  11  10  3\nf  19  11  3\nf  11  1  5\nf  10  11  5\nf  20  10  5\nf  20  9  8\nf  10  20  8\nf  18  10  8\nf  9  16  7\nf  8  9  7\nf  14  8  7\nf  12  15  6\nf  13  12  6\nf  17  13  6\nf  13  1  11\nf  12  13  11\nf  19  12  11\n";

	return 0;
}

ChSharedPtr<ChBody> createSphere(ChSystem& mphysicalSystem, double radius, ChVector<> position, ChColor color, double mass)
{
	// create the body
	ChSharedPtr<ChBody> sphere(new ChBody);
	sphere->SetPos(position);
	sphere->SetMass(mass);
	mphysicalSystem.Add(sphere);

	double c = 1.0/sqrt(2.0);

	// Define the collision shape
	sphere->GetCollisionModel()->ClearModel();
	sphere->GetCollisionModel()->AddSphere(radius);
	sphere->GetCollisionModel()->BuildModel();
	sphere->SetCollide(true);

	// Add visualization geometry
	ChSharedPtr<ChSphereShape> sphereShape1(new ChSphereShape);
	sphereShape1->GetSphereGeometry().rad = radius;
	sphere->AddAsset(sphereShape1);

	// Add color
	ChSharedPtr<ChVisualization> sphereColor(new ChVisualization);
	sphereColor->SetColor(color);
	sphere->AddAsset(sphereColor);

	return sphere;
}

ChSharedPtr<ChBody> createRock(ChSystem& mphysicalSystem, std::string filename, ChVector<> position, ChQuaternion<> rotation, ChColor color, double mass)
{
	// create the body
	ChSharedPtr<ChBody> rock(new ChBody);
	rock->SetPos(position);
	rock->SetRot(rotation);
	rock->SetMass(mass);
	mphysicalSystem.Add(rock);

	// Define the collision shape
	ChTriangleMeshConnected trimesh;
	trimesh.LoadWavefrontMesh(filename, false, false);
	rock->GetCollisionModel()->ClearModel();
	rock->GetCollisionModel()->AddTriangleMesh(trimesh, false, true);
	rock->GetCollisionModel()->BuildModel();
	rock->SetCollide(true);

	// Add visualization geometry
	ChSharedPtr<ChAssetLevel> mlevelA(new ChAssetLevel);
	ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
	mobjmesh->SetFilename(filename);
	mlevelA->AddAsset(mobjmesh);
	rock->AddAsset(mlevelA);

	// Add color
	ChSharedPtr<ChVisualization> rockColor(new ChVisualization);
	rockColor->SetColor(color);
	rock->AddAsset(rockColor);

	return rock;
}

ChSharedPtr<ChBody> createShearPlate(ChSystem& mphysicalSystem, ChVector<> size, double TH, ChVector<> position, ChQuaternion<> rotation, ChColor color)
{
	double L = size.x;
	double H = size.y;
	double W = size.z;

	// create the body
	ChSharedPtr<ChBody> shearPlate(new ChBody);
	shearPlate->SetPos(position);
	shearPlate->SetRot(rotation);
	mphysicalSystem.Add(shearPlate);

	// Define the collision shape
	shearPlate->GetCollisionModel()->ClearModel();
	shearPlate->GetCollisionModel()->AddBox((L+2*TH)*.5,H*.5,TH*.5, ChVector<>(0,0,.5*W+.5*TH));
	shearPlate->GetCollisionModel()->AddBox((L+2*TH)*.5,H*.5,TH*.5, ChVector<>(0,0,-.5*W-.5*TH));
	shearPlate->GetCollisionModel()->AddBox(TH*.5,H*.5,W*.5, ChVector<>(.5*L+.5*TH,0,0));
	shearPlate->GetCollisionModel()->AddBox(TH*.5,H*.5,W*.5, ChVector<>(-.5*L-.5*TH,0,0));
	shearPlate->GetCollisionModel()->BuildModel();
	shearPlate->SetCollide(true);

	// Add visualization geometry
	ChSharedPtr<ChBoxShape> boxShape1(new ChBoxShape);
	boxShape1->GetBoxGeometry().Pos = ChVector<>(0,0,.5*W+.5*TH);
	boxShape1->GetBoxGeometry().Size = ChVector<>((L+2*TH)*.5,H*.5,TH*.5);
	shearPlate->AddAsset(boxShape1);

	ChSharedPtr<ChBoxShape> boxShape2(new ChBoxShape);
	boxShape2->GetBoxGeometry().Pos = ChVector<>(0,0,-.5*W-.5*TH);
	boxShape2->GetBoxGeometry().Size = ChVector<>((L+2*TH)*.5,H*.5,TH*.5);
	//shearPlate->AddAsset(boxShape2);

	ChSharedPtr<ChBoxShape> boxShape3(new ChBoxShape);
	boxShape3->GetBoxGeometry().Pos = ChVector<>(.5*L+.5*TH,0,0);
	boxShape3->GetBoxGeometry().Size = ChVector<>(TH*.5,H*.5,W*.5);
	shearPlate->AddAsset(boxShape3);

	ChSharedPtr<ChBoxShape> boxShape4(new ChBoxShape);
	boxShape4->GetBoxGeometry().Pos = ChVector<>(-.5*L-.5*TH,0,0);
	boxShape4->GetBoxGeometry().Size = ChVector<>(TH*.5,H*.5,W*.5);
	shearPlate->AddAsset(boxShape4);

	// Add color
	ChSharedPtr<ChVisualization> plateColor(new ChVisualization);
	plateColor->SetColor(color);
	shearPlate->AddAsset(plateColor);

	return shearPlate;
}

ChSharedPtr<ChBody> createBox(ChSystem& mphysicalSystem, ChVector<> size, ChVector<> position, ChQuaternion<> rotation, ChColor color)
{
	double L = size.x;
	double H = size.y;
	double W = size.z;

	// create the body
	ChSharedPtr<ChBody> box(new ChBody);
	box->SetPos(position);
	box->SetRot(rotation);
	mphysicalSystem.Add(box);

	// Define the collision shape
	box->GetCollisionModel()->ClearModel();
	box->GetCollisionModel()->AddBox(L*.5,H*.5,W*.5);
	box->GetCollisionModel()->BuildModel();
	box->SetCollide(true);

	// Add visualization geometry
	ChSharedPtr<ChBoxShape> boxShape1(new ChBoxShape);
	boxShape1->GetBoxGeometry().Pos = ChVector<>(0,0,0);
	boxShape1->GetBoxGeometry().Size = ChVector<>(L*.5,H*.5,W*.5);
	box->AddAsset(boxShape1);

	// Add color
	ChSharedPtr<ChVisualization> boxColor(new ChVisualization);
	boxColor->SetColor(color);
	box->AddAsset(boxColor);

	return box;
}

class SoilBin {
public:
	// THE DATA
	double L;
	double H;
	double W;
	double TH;
	double L_plate;
	double H_plate;
	double W_plate;
	double desiredVelocity;

	double nextFillTime;
	double sphereVolume;

	// bodies
	ChSharedPtr<ChBody> ground;
	ChSharedPtr<ChBody> bottom;
	ChSharedPtr<ChBody> plate;
	ChLinkLockLock* translational;

	// THE FUNCTIONS

	// Build and initialize the car, creating all bodies corresponding to
	// the various parts and adding them to the physical system - also creating
	// and adding constraints to the system.
	SoilBin(ChSystem&  mphysicalSystem,	///< the chrono::engine physical system 
		double L, double H, double W, double TH, double L_plate, double H_plate, double W_plate, double desiredVelocity, double mu)
	{
		this->L = L;
		this->H = H;
		this->W = W;
		this->TH = TH;
		this->L_plate = L_plate;
		this->H_plate = H_plate;
		this->W_plate = W_plate;
		this->desiredVelocity = desiredVelocity;

		nextFillTime = 0;
		sphereVolume = 0;

		ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
		mmaterial->SetFriction(mu); // Friction coefficient of steel

		// Create ground
		ground = createBox(mphysicalSystem, ChVector<>(L+2*TH,H,W+2*TH), ChVector<>(0,-H,0), ChQuaternion<>(1,0,0,0), ChColor(0.6,0.6,0.6));
		ground->SetMaterialSurface(mmaterial);
		ground->SetBodyFixed(true);
		ground->GetCollisionModel()->SetFamily(4);
		ground->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
		bodyTypes.push_back("ground");

		// Create bottom
		bottom = createShearPlate(mphysicalSystem, ChVector<>(L,H,W), TH, ChVector<>(0,  0, 0), ChQuaternion<>(1,0,0,0), ChColor(0.6,0.3,0.6));
		bottom->SetMaterialSurface(mmaterial);
		bottom->GetCollisionModel()->SetFamily(4);
		bottom->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
		bottom->SetBodyFixed(true);
		bodyTypes.push_back("bottom");

		// Create plate
		plate = createBox(mphysicalSystem, ChVector<>(L_plate, H_plate, W_plate), ChVector<>(0,0.5*H+0.5*H_plate,0), ChQuaternion<>(1,0,0,0), ChColor(0.3,0.5,0.3));
		plate->SetMaterialSurface(mmaterial);
		plate->GetCollisionModel()->SetFamily(4);
		plate->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
		bodyTypes.push_back("plate");

		// create the translational joint between the top shear box and weight load
		ChVector<> plateCM = plate->GetPos();
		translational = new ChLinkLockLock();
		translational->Initialize(ground, plate, 
			ChCoordsys<>(plateCM, QUNIT) );//chrono::Q_from_AngAxis(-CH_C_PI/2,VECT_Y)) );
		mphysicalSystem.AddLink(translational);

		// apply motion
		ChFunction_Ramp* motionFunc = new ChFunction_Ramp(0,desiredVelocity);
		translational->SetMotion_Y(motionFunc);
	}

	int SoilBin::setPlateSpeed(double startPos, double desiredVelocity)
	{
		ChFunction_Ramp* motionFunc = new ChFunction_Ramp(startPos,desiredVelocity);
		translational->SetMotion_Y(motionFunc);
		return 0;
	}
};

double setParticleDensity(ChSystem& mphysicalSystem,double particleDensity,double radius)
{
	int sphereIndex = 0;
	std::vector<ChBody*>::iterator abody = mphysicalSystem.Get_bodylist()->begin();
	double totalMass = 0;
	while (abody != mphysicalSystem.Get_bodylist()->end())
	{
		ChBody* bpointer = (*abody);

		if(bpointer->GetCollisionModel()->GetShapeType() == collision::SPHERE || bpointer->GetCollisionModel()->GetShapeType() == collision::TRIANGLEMESH )
		{
			//double radius = particleRadii[sphereIndex];
			double volume = 4.0*CH_C_PI*radius*radius*radius/3.0;
			double mass = particleDensity*volume;
			bpointer->SetMass(mass);
			totalMass+=mass;
			sphereIndex++;
		}

		abody++;
	}

	return totalMass;
}

int saveSystemState(std::string filename, ChSystem& mphysicalSystem)
{
	chrono::ChStreamOutAsciiFile saveStream(filename.c_str());

	std::vector<ChBody*>::iterator abody = mphysicalSystem.Get_bodylist()->begin();
	int bodyNum = 0;
	int sphereIndex = 0;
	while (abody != mphysicalSystem.Get_bodylist()->end())
	{
		chrono::Vector bodyAngs;
		chrono::Quaternion bodyRot;
		ChBody* bpointer = (*abody);
		bodyRot = bpointer->GetRot();
		bodyAngs = bodyRot.Q_to_NasaAngles();

		saveStream << bpointer->GetPos().x << ", ";
		saveStream << bpointer->GetPos().y << ", ";
		saveStream << bpointer->GetPos().z << ", ";
		saveStream << bpointer->GetRot().e0 << ", ";
		saveStream << bpointer->GetRot().e1 << ", ";
		saveStream << bpointer->GetRot().e2 << ", ";
		saveStream << bpointer->GetRot().e3 << ", ";
		saveStream << bpointer->GetPos_dt().x << ", ";
		saveStream << bpointer->GetPos_dt().y << ", ";
		saveStream << bpointer->GetPos_dt().z << ", ";
		if(bodyNum == 0) saveStream << "1" << ", ";
		else if(bodyNum == 1) saveStream << "2" << ", ";
		else if(bodyNum == 2) saveStream << "3" << ", ";
		else if(bodyNum == 3) saveStream << "4" << ", ";
		else saveStream << "0" << ", ";

		if(bodyNum<4)
		{
			saveStream << "0" << ", ";
		}
		else
		{
			saveStream << particleRadii[sphereIndex] << ", ";
			sphereIndex++;
		}
		saveStream << "\n";

		abody++;
		bodyNum++;
	}

	return 0;
}

bool checkFilled(ChSystem& mphysicalSystem, SoilBin* soilBin, double r)
{
	bool filled = false;
	std::vector<ChBody*>::iterator abody = mphysicalSystem.Get_bodylist()->begin();
	while (!filled && abody != mphysicalSystem.Get_bodylist()->end())
	{
		ChBody* bpointer = (*abody);
		if(bpointer->GetCollisionModel()->GetShapeType() == collision::SPHERE || bpointer->GetCollisionModel()->GetShapeType() == collision::TRIANGLEMESH)
		{
			if(bpointer->GetPos().y >= soilBin->H*0.5-1.1*r) filled = true;
		}
		abody++;
	}

	return filled;
}

int fill(ChSystem& mphysicalSystem, SoilBin* soilBin, double r, std::string rockFile, ChIrrApp* application, bool visualize, double gravity, double desiredBulkDensity, double desiredVelocity)
{
	printf("Time: %.3f (Stage 1): Num. Bodies = %d\n", mphysicalSystem.GetChTime(), mphysicalSystem.GetNbodies());

	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	mphysicalSystem.SetIterLCPmaxItersStab(20);

	soilBin->setPlateSpeed(0,0);	

	int nLength = (int) (0.5*soilBin->L/r);
	int nWidth = (int) (0.5*soilBin->W/r);
	ChSharedPtr<ChBody> rock;
	// Check to see if box is already filled
	bool filled = false;
	if(mphysicalSystem.GetChTime()>=soilBin->nextFillTime) filled = checkFilled(mphysicalSystem, soilBin, r);

	if(!filled&&mphysicalSystem.GetChTime()>=soilBin->nextFillTime)
	{
		soilBin->nextFillTime += sqrt(fabs(4*r/gravity));
		for(int i=0;i<nLength;i++) {
			for(int j=0;j<nWidth;j++) {
				rock = createRock(mphysicalSystem, rockFile, ChVector<>(2*i*r+r-r*nLength,soilBin->H*0.5-r,2*j*r+r-r*nWidth), ChQuaternion<>(1,0,0,0), ChColor(1,1,1), 1);
				particleRadii.push_back(r);
				soilBin->sphereVolume += 4.0*CH_C_PI*r*r*r/3.0;
				rock->SetPos_dt(ChVector<>(0,-100,0));
				if(visualize) {
					application->AssetBind(rock);
					application->AssetUpdate(rock);
				}
			}
		}
	}

	int nextStage = 1;
	if(filled) {
		double particleDensity = desiredBulkDensity/(soilBin->sphereVolume/(soilBin->L*soilBin->W*(soilBin->H-r)));
		double totalMass = setParticleDensity(mphysicalSystem,particleDensity,r);
		soilBin->setPlateSpeed(-mphysicalSystem.GetChTime()*desiredVelocity,desiredVelocity);
		nextStage = 2;
	}
	return nextStage;
}

int compress(ChSystem& mphysicalSystem, SoilBin* soilBin, ChIrrApp* application, bool visualize, int maxiterations)
{
	mphysicalSystem.SetIterLCPmaxItersSpeed(maxiterations);
	mphysicalSystem.SetIterLCPmaxItersStab(maxiterations);

	//forceOnGround.x = 0;
	//forceOnGround.y = 0;
	//forceOnGround.z = 0;
	//_reporter_class my_contact_reporter;
	//my_contact_reporter.bodyToGetForceOn = shearBox->cieling;//shearBox->ground;
	//my_contact_reporter.msystem = &mphysicalSystem;
	//mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_reporter);

	double L = soilBin->L_plate;
	double W = soilBin->W_plate;
	printf("Time: %.3f (Stage 2): Ave. vertical speed: %.3f m/s, Ground Pressure: (%.3f, %.3f, %.3f)\n", mphysicalSystem.GetChTime(), soilBin->plate->GetPos_dt().y,soilBin->translational->Get_react_force().x/(L*W),soilBin->translational->Get_react_force().y/(L*W),soilBin->translational->Get_react_force().z/(L*W));

	int nextStage = 2;
	//if(fabs(shearBox->sumCielingVelocity/shearBox->nCielingVelocity) < 1 || mphysicalSystem.GetChTime()>1) {
	//	shearBox->setShearSpeed(-mphysicalSystem.GetChTime()*desiredVelocity,desiredVelocity);
	//	nextStage = 3;
	//}

	//shearBox->sumCielingVelocity += shearBox->cieling->GetPos_dt().y;
	//shearBox->nCielingVelocity++;

	return nextStage;
}
/*
int shear(ChSystem& mphysicalSystem, ShearBox* shearBox, double lengthToRun)
{
	printf("Time: %.3f (Stage 3): Speed: %.3f m/s Force: (%.3f, %.3f, %.3f) Pos: (%.3f, %.3f, %.3f)\n", mphysicalSystem.GetChTime(), shearBox->top->GetPos_dt().x, shearBox->translational->Get_react_force().x, shearBox->translational->Get_react_force().y, shearBox->translational->Get_react_force().z, shearBox->top->GetPos().x, shearBox->top->GetPos().y, shearBox->top->GetPos().z);

	int nextStage = 3;
	if(fabs(shearBox->top->GetPos().x)>fabs(lengthToRun)) nextStage = 4;
	return nextStage;
}
*/
int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO... 
	//
	bool visualize = atoi(argv[1]);
	double particleRadius = atof(argv[2]); // [cm]
	double normalPressure = atof(argv[3]); 	// apply normal force to cieling// 16,888.1 Pa // 44,127.0 Pa// 71,365.9 Pa
	double particleDensity = atof(argv[4]); // [g/cm^3]
	double muParticles = atof(argv[5]);
	double muParticlesRolling = atof(argv[6]);
	double muParticlesSpinning = atof(argv[7]);
	double cohesion = atof(argv[8]);
	double hh = atof(argv[9]); // [s]
	double tolerance = atof(argv[10]);
	int maxiterations = atoi(argv[11]);
	int useAPGD = atoi(argv[12]);
	int useWarmStarting = atoi(argv[13]);
	std::string data_folder = argv[14];

	// working in [cm, g, s]
	double gravity = -981; // [cm/s^2];
	double L = 40; // [cm]
	double H = 16; // [cm]
	double W = 15; // [cm]
	double TH = 3; // [cm]
	double L_plate = 13;  // [cm]
	double H_plate = 1; // [cm]
	double W_plate = 5; // [cm]
	double desiredVelocity = 1; // [cm/s]
	double desiredBulkDensity = 1.389475; // [g/cm^3]
	double lengthToRun = 0.5; // [cm]
	double muWalls = 0;
	int saveModulus = ((int)(1.0/hh))/100; // 100 fps
	cielingHeight = 0;

	// Initialize the shear stress vs. displacement output file
	stringstream dataStream;
	dataStream << data_folder << "/pressureSinkage_" << visualize << "_" << particleRadius << "_" << normalPressure << "_" << particleDensity << "_" << muParticles << "_" << muParticlesRolling << "_" << muParticlesSpinning << "_" << cohesion << "_" << hh << "_" << tolerance << "_" << maxiterations << "_" << useAPGD << "_" << useWarmStarting << ".csv";
	ChStreamOutAsciiFile pressureData(dataStream.str().c_str());

	// Create the shear box (with no shearing motion and the cieling locked)
	SoilBin* soilBin = new SoilBin(mphysicalSystem, L, H, W, TH, L_plate, H_plate, W_plate, desiredVelocity, muWalls);

	// Prepare the physical system for the simulation 
	mphysicalSystem.Set_G_acc(ChVector<>(0,gravity,0));
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	if(useAPGD) mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_APGD);
	mphysicalSystem.SetTol(tolerance);
	mphysicalSystem.SetTolSpeeds(tolerance);
	mphysicalSystem.SetUseSleeping(false);
	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);
	mphysicalSystem.SetMaxPenetrationRecoverySpeed(1600000000000); // used by Anitescu stepper only
	mphysicalSystem.SetIterLCPmaxItersSpeed(maxiterations);
	mphysicalSystem.SetIterLCPmaxItersStab(maxiterations); // unuseful for Anitescu, only Tasora uses this
	mphysicalSystem.SetIterLCPwarmStarting(useWarmStarting);
	chrono::ChLcpIterativeSolver* msolver_speed = (chrono::ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
	msolver_speed->SetRecordViolation(true);

	// Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
	ChIrrApp* application;
	if(visualize)
	{
		// 1920x1080 // 800x600
		application = new ChIrrApp(&mphysicalSystem, L"Shear test",core::dimension2d<u32>(1920,1080),false, false);

		application->AddTypicalLogo();
		application->AddTypicalSky();
		application->AddTypicalLights();
		application->AddTypicalCamera(core::vector3df(0,1,-30));

		application->SetStepManage(false);

		application->AssetBindAll();
		application->AssetUpdateAll();
	}

	// THE TIME CYCLE
	int timeIndex = 0;
	int frameIndex = 0;
	int currentStage = 1;
	double nextFillTime = 0;

	stringstream rockStream;
	rockStream << data_folder << "/rock.obj";
	generateRockObject(rockStream.str(),particleRadius,0);

	while((!visualize||application->GetDevice()->run())&&currentStage<=2)
	{
		// Switch between the direct shear stages
		switch(currentStage)
		{
		case 1:
			// Stage 1: Fill
			currentStage = fill(mphysicalSystem,soilBin,particleRadius,rockStream.str(),application,visualize,gravity,desiredBulkDensity,desiredVelocity);
			break;
		case 2:
			// Stage 2: Compress
			pressureData << mphysicalSystem.GetChTime() << ", " << soilBin->plate->GetPos().x << ", " << soilBin->plate->GetPos().y << ", " << soilBin->plate->GetPos().z << ", " << soilBin->translational->Get_react_force().x << ", " << soilBin->translational->Get_react_force().y << ", " << soilBin->translational->Get_react_force().z << ", " << soilBin->plate->GetPos_dt().x << ", " << soilBin->plate->GetPos_dt().y << ", " << soilBin->plate->GetPos_dt().z << ", \n";
			currentStage = compress(mphysicalSystem,soilBin,application,visualize,maxiterations);
			break;
		case 3:
			// Stage 3: Shear
			//shearData << mphysicalSystem.GetChTime() << ", " << shearBox->top->GetPos().x << ", " << shearBox->top->GetPos().y << ", " << shearBox->top->GetPos().z << ", " << shearBox->translational->Get_react_force().x << ", " << shearBox->translational->Get_react_force().y << ", " << shearBox->translational->Get_react_force().z << ", " << shearBox->top->GetPos_dt().x << ", " << shearBox->top->GetPos_dt().y << ", " << shearBox->top->GetPos_dt().z << ", \n";
			//currentStage = shear(mphysicalSystem,shearBox,lengthToRun);
			break;
		}

		// performe the time step
		if(visualize)
		{
			application->BeginScene();

			application->DrawAll();

			application->EndScene();
		}
		mphysicalSystem.DoStepDynamics(hh);
		
		// save the positions of the bodies
		if(timeIndex%saveModulus==0)
		{
			stringstream ss;
			ss << data_folder << "/pos" << frameIndex << ".dat";
			saveSystemState(ss.str(), mphysicalSystem);
			frameIndex++;
		}

		timeIndex++;
	}

	DLL_DeleteGlobals();

	return 0;
}