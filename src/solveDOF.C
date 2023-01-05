/*---------------------------------------------------------------------------*\
Writer

	Abdurrahman Gazi Yavuz

Application

    solveDOF

Description

    A Low Fidelity Store Separation Tool
	
		Solving 6DOF Equations on a store using
		its Aerodynamic database given in a lookupTable
		form.
		
		It also uses flow field of the Parent object of 
		the store (i.e. an Aircraft ejecting it) for downwash effect
		
		6DOF motion dict given in solveDOFDict file

\*---------------------------------------------------------------------------*/

#include "IFstream.H"
#include "argList.H"
#include "Time.H"
#include "fvCFD.H"
#include "sixDoFRigidBodyMotion.H"
#include "multiDimInterpolate.H"
#include "missile.H"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

int main(int argc, char *argv[])
{
    #include "setRootCase.H"
    #include "createTime.H"
	
	/*
	// Prepare probe dictionary name, ifstream to read dict and dictionary
	Foam::word probeFileName("probesDict");
	
	// Construct probeFile from pathname (see -> explicit IFstream(const fileName& pathname, IOstreamOption streamOpt = IOstreamOption()) in IFstream.H)
	Foam::IFstream 	probeFile(probeFileName);
	
	// Construct probeDict from Istream (see -> dictionary(Istream& is) in dictionary.H)
	Foam::dictionary probeDict(probeFileName);
	*/
	
	// Assign solveDOFDict as a word
	Foam::word solveDOFDictName("system/solveDOFDict");
	
	// Read the solveDOFDict with IFstream
	Foam::IFstream 	solveDOFDictFile(solveDOFDictName);
	
	// Construct dictionary from Istream (see -> dictionary(Istream& is) in dictionary.H)
	Foam::dictionary solveDOFDict(solveDOFDictFile);
	
    //- Construct from constant and state dictionaries (see -> sixDoFRigidBodyMotion(const dictionary& dict, const dictionary& stateDict, const Time& time) in sixDoFRigidBodyMotion.H)
	Foam::sixDoFRigidBodyMotion SDRBM
	(
		solveDOFDict,
		solveDOFDict,
		runTime
	);
	
	//
	Foam::word databaseName("eglinDB");
	Foam::IFstream 	databaseFile(databaseName);
	
	missile<Foam::doubleScalar> mis 
	(
	solveDOFDict,
	runTime
	);
	
	// Construct from a RectangularMatrix of doubleScalars from Istream (see -> explicit Matrix(Istream& is) in Matrix.H)
	Foam::RectangularMatrix<Foam::doubleScalar> database(databaseFile);
	
	//multiDimInterpolate<Foam::doubleScalar> mdi("eglinDB");
	multiDimInterpolate<Foam::doubleScalar> mdi("eglinDB");
	
	mdi.interpolate(0.85001, 0.0001, 0.0001);
	//mdi.upper_band(0.9, 12., 7.);
	
	Foam::Info<< " cfx : " << mdi.cfx() << Foam::endl;
	
	//mdi.corner_values();
	
	Foam::Info<< " cfy : " << mdi.cfy() << Foam::endl;
	
	// DECLERATE INTERPOLATION OBJECT HERE
	
	//Foam::Info<< " database : " << database << Foam::endl;
	
	// Acces rows (m) and columns (n) of the Matrix (see -> inline label m()/n() const noexcept in Matrix.H)
	Foam::Info<< " database m : " << database.m() << Foam::endl;
	Foam::Info<< " database n : " << database.n() << Foam::endl;

	// See polyMesh.C -> namespace Foam { word polyMesh::defaultRegion = "region0";}
	Foam::word regionName = Foam::fvMesh::defaultRegion;
	
	// Latest time in airfoil2D case
	scalar latestTime     = 0;
	
	// Set time to latestTime for airfoil2D case (see -> virtual void setTime(const scalar newTime, const label newIndex) in Time.H)
	runTime.setTime(latestTime, latestTime);
	/*
	// Info
    Foam::Info<< " latestTime : " << latestTime << Foam::endl;
	
	// Create fvMesh object from IOobject for present case  (see -> explicit fvMesh(const IOobject& io) in fvMesh.H)
	Foam::fvMesh mesh
	(
		Foam::IOobject
		(
			regionName,
			runTime.constant(),
			runTime,
			Foam::IOobject::MUST_READ,
			Foam::IOobject::NO_WRITE,
			false
		)
	);

	// Create pressure field in mesh for present time latestTime from Istream
	// (see -> DimensionedField(const IOobject& io, const Mesh& mesh, const word& fieldDictEntry = "value" ) in DimensionedField.H)
	Foam::volScalarField p
	(
		Foam::IOobject
		(
			"p",
			runTime.timeName(),
			mesh,
			Foam::IOobject::MUST_READ,
			Foam::IOobject::AUTO_WRITE
		),
		mesh
	);

	// Create velocity field in mesh for present time latestTime from Istream
	// (see -> DimensionedField(const IOobject& io, const Mesh& mesh, const word& fieldDictEntry = "value" ); in DimensionedField.H)
	Foam::volVectorField U
	(
		Foam::IOobject
		(
			"U",
			runTime.timeName(),
			mesh,
			Foam::IOobject::MUST_READ,
			Foam::IOobject::AUTO_WRITE
		),
		mesh
	);
	
	// Define a point in domain
	Foam::point x(50, 10, 0.025);
	
	// Find nearest cell to point x (see -> label findNearestCell(const point& location) const in primitiveMesh.H)
	Foam::label celli 	= mesh.findNearestCell(x);
	
	// Find cell enclosing point x (-1 if not in mesh) (see -> label findCell(const point& location) const in primitiveMesh.H)
	Foam::label celli2 	= mesh.findCell(x);
	
    Foam::Info<< " nearest cell to point x : " << celli << Foam::endl;
	
    Foam::Info<< " cell enclosing point x : " << celli2 << Foam::endl;
	
    Foam::Info<< " Pressure path : " << p.filePath() << Foam::endl;
	
    Foam::Info<< " Pressure at celli : " 	 << p[celli] << Foam::endl;
	
    //Foam::Info<< " Pressure : " 	 << p << Foam::endl;
	
    //Foam::Info<< " internalField : "  << mesh.lookupObject<volScalarField>("p") << Foam::endl;
	
    Foam::Info<< " timeName : "  << runTime.timeName() << Foam::endl;
	
    Foam::Info<< " runTimeName : "  << runTime.name() << Foam::endl;
	
    Foam::Info<< " meshName : "  << mesh.name() << Foam::endl;
		
	// Create probe object
	
	probes probe
	(
		Foam::fvMesh::defaultRegion,
		runTime,
		probeDict,
		false,
		false
	);
	// Check if probe constructed well
    Foam::Info<< " Fields to sample : " << probe.fieldNames() << Foam::endl;
	*/
	
	Foam::scalar fx(100.);
	Foam::scalar fy(0.);
	Foam::scalar fz(0.);
	
	Foam::vector f(fx, fy, fz);
	Foam::vector m(fx, fy, fz);
	
	Foam::scalar deltaT 	= runTime.deltaTValue();
	
	Foam::Info << Foam::nl << Foam::nl << Foam::nl << Foam::endl;
		
	while(runTime.loop())
	{
		
		Foam::Info << "Timestep : " << runTime.value() << Foam::nl << Foam::endl;
		mis.update(deltaT);
		// Update SDRBM state (see ->  void update (bool firstIter, const vector& fGlobal, const vector& tauGlobal, scalar deltaT, scalar deltaT0) in sixDoFRigidBodyMotion.H)
		/*
		SDRBM.update
		(
			false,
			f,
			m,
			deltaT,
			deltaT
		);
		*/
		// Print out the state of SDRBM (see -> inline const sixDoFRigidBodyMotionState& state() const in sixDoFRigidBodyMotion.H)
		//SDRBM.state
		
		
		// Update motion0_ to motion_ (see -> inline void newTime() in sixDoFRigidBodyMotion.H)
		//SDRBM.newTime();
		
	}
	/*
	Foam::pointField points 	= mesh.points();
	
    Foam::Info<< " nCells = " << mesh.nCells() << Foam::endl;
    Foam::Info<< " nFaces = " << mesh.nFaces() << Foam::endl;
    Foam::Info<< " nPoints = " << mesh.nPoints() << Foam::endl;
    Foam::Info<< " nInternalFaces = " << mesh.nInternalFaces() << Foam::endl;
    Foam::Info<< " name = " << mesh.name() << Foam::endl;
    */
    Foam::Info<< "\nEnd\n" << endl;

    return 0;
}


// ************************************************************************* //
