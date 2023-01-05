#include "sixDoFRigidBodyMotion.H"
#include "fvCFD.H"

class Atmosphere
{
private:
	Foam::doubleScalar p_, rho_, T_, soundSpeed_;
	
	Foam::doubleScalar altitude_;
	
	Foam::doubleScalar p0_ 	= 101325.;		// Reference Pressure (Pa)
	Foam::doubleScalar T0_ 	= 288.15;		// Reference Temperature (K)
	Foam::doubleScalar g 		= 9.80665;		// Earth's gravity (m/s2)
	Foam::doubleScalar M 		= 0.0289644;		// Molar mass of dry air (kg/mol )
	Foam::doubleScalar R0 		= 8.314462618;		// Universal gas constant (J/(mol·K))
	Foam::doubleScalar R 		= 287.; 		
	Foam::doubleScalar gamma 	= 1.4;
	
	Foam::dictionary dict_;
	
public:
	Atmosphere(){};
	
	Atmosphere
	(
		Foam::dictionary dict
	)
	:
	dict_(dict)
	{
		altitude_ 		= dict_.get<Foam::doubleScalar>("altitude");
		this -> initialize();
	}
	
	void pressure
	(
		Foam::doubleScalar altitude
	)
	{
		p_ 	= p0_ * Foam::exp(- g * altitude * M / T0_ / R0);
	
	}
	
	void temperature
	(
		Foam::doubleScalar altitude
	)
	{
		
		if (altitude < 11000.)
		{
			T_ 	= T0_ - 0.0065 * altitude;
		}
		else if (altitude < 20000.)
		{
			T_ 	= 216.65;
		}
		else
		{
			T_ 	= 216.65;
			Foam::Info << "Current Altitude" << Foam::nl;
			Foam::Info << altitude << " m" << Foam::nl;
			Foam::Info << "too much" << Foam::nl;
		}
	}
	
	void density
	(
		Foam::doubleScalar altitude
	)
	{
		rho_ 	= p_ / T_ / R;
	}
	
	void sound
	(
		Foam::doubleScalar altitude
	)
	{		
		soundSpeed_	= Foam::sqrt(gamma * R * T_);
	}
	
	void update
	(
		Foam::doubleScalar newAltitude
	)
	{
		altitude_ 	= newAltitude;
		
		this -> pressure(altitude_);
		this -> temperature(altitude_);
		this -> density(altitude_);
		this -> sound(altitude_);
	}
	
	void initialize()
	{
		
		this -> pressure(altitude_);
		this -> temperature(altitude_);
		this -> density(altitude_);
		this -> sound(altitude_);
	}
	
	// Access
	
	inline Foam::doubleScalar pressure()
	{
		return p_;
	}
	
	inline Foam::doubleScalar temperature()
	{
		return T_;
	}
	
	inline Foam::doubleScalar density()
	{
		return rho_;
	}
	
	inline Foam::doubleScalar soundSpeed()
	{
		return soundSpeed_;
	}
	
	
};
template<typename T>
class Wind
:
public Atmosphere
{
private:

	T windMach_;
	T windSpeed_;
	T altitude_;
	Foam::Vector<T> windDirection_;
	Foam::Vector<T> windVelocity_;
	Foam::dictionary dict_;
	
public:

	Wind(){};
	
	Wind
	(
		Foam::dictionary dict
	)
	:
	dict_(dict)
	{
		altitude_ 	= dict.get<T>("altitude");
		
		this -> initialize();
				
		windMach_ 	= dict.get<T>("windMach");
		windDirection_	= dict.get<Foam::Vector<T>>("windDirection");
		
		windDirection_.normalise();
		
		windSpeed_ 	= this -> soundSpeed() * windMach_;
		windVelocity_ 	= windDirection_ * windSpeed_;
	};
	
	void update
	(
		T newAltitude
	)
	{
		altitude_ 	= newAltitude;
		
		this -> pressure(altitude_);
		this -> temperature(altitude_);
		this -> density(altitude_);
		this -> sound(altitude_);
		
		windSpeed_ 	= this -> soundSpeed() * windMach_;
		windVelocity_ 	= windDirection_ * windSpeed_;
	}
	
	inline Foam::Vector<T> velocity()
	{
		return windVelocity_;
	}
	
	inline Foam::Vector<T> direction()
	{
		return windDirection_;
	}
	
};

template<typename T>
class missile
{
	Foam::doubleScalar g 		= 9.80665;		// Earth's gravity (m/s2)
	
	Foam::sixDoFRigidBodyMotion SDRBM_;
	
	multiDimInterpolate<T> MDI_;
	
	Atmosphere ATM_;
	
	Wind<T> W_;
	
	Foam::Tensor<T> I_;
	
	Foam::Tensor<T> bodyToNED_;
	
	Foam::Tensor<T> rateToEul_;
	
	Foam::Vector<T> position_;	// *.x() -> x, *.y() -> y, *.z() -> z
	
	Foam::Vector<T> attitude_;	// *.x() -> phi, *.y() -> theta, *.z() -> psi
	
	Foam::Vector<T> rate_;		//  *.x() -> p, *.y() -> q, *.z() -> r
	
	Foam::Vector<T> velocity_;	//  *.x() -> u, *.y() -> v, *.z() -> w
	
	T	mach_, aoa_, aos_;
	
	Foam::Vector<T> relVel_;	//  *.x() -> u, *.y() -> v, *.z() -> w
	
	Foam::Vector<T> Fgra_;	//  *.x() -> gx, *.y() -> gy, *.z() -> gz
	
	Foam::Vector<T> Faero_, Fcorio_, Fext_, Fsum_;
	
	Foam::Vector<T> Maero_, Mcorio_, Mext_, Msum_;
	
	Foam::dictionary dict_;
	
	Foam::diagTensor Itensor_;
	
public:

	missile(const dictionary& dict, const Time& time)
	:
	SDRBM_
	(
	    dict,
	    dict,
	    time
	),
	dict_(dict),
	MDI_(dict),
	W_(dict),
	ATM_(dict),	
	attitude_(dict.getOrDefault<Vector<T>>("attitude", Foam::vector::zero)),
	position_(dict.get<Vector<T>>("centreOfMass")),
	rate_(dict.getOrDefault<Vector<T>>("initialRate", Foam::vector::zero)),
	relVel_(Foam::vector::zero),
	velocity_(Foam::vector::zero),
	Faero_(Foam::vector::zero),
	Fcorio_(Foam::vector::zero),
	Fext_(Foam::vector::zero),
	Fsum_(Foam::vector::zero),
	Maero_(Foam::vector::zero),
	Mcorio_(Foam::vector::zero),
	Mext_(Foam::vector::zero),
	Msum_(Foam::vector::zero),
	Itensor_(dict.get<diagTensor>("momentOfInertia")),
	I_(dict.get<Tensor<T>>("inertia")),
	bodyToNED_(Foam::Tensor<T>::zero),
	rateToEul_(Foam::Tensor<T>::zero)
	{
	
		this -> relativeVelocity();
		this -> scalarMach();
		this -> scalarAOA();
		this -> scalarAOS();
		
		this -> database();
		
		this -> gravitationalForces();
		this -> corioringForces();
		this -> aerodynamicForces();
		this -> externalForces();
		
		this -> sumForces();
		
		this -> corioringMoments();
		this -> aerodynamicMoments();
		this -> externalMoments();
		
		this -> sumMoments();
		
	};
	
	inline void gravitationalForces()
	{
		T gx 		=  - g * Foam::sin(attitude_.y());
		T gy 		=    g * Foam::sin(attitude_.x()) * Foam::cos(attitude_.y());
		T gz 		=    g * Foam::cos(attitude_.x()) * Foam::cos(attitude_.y());
		
		Foam::Vector<T> gravity(gx, gy, gz);
		
		Fgra_ 	= gravity;
		
	}
	
	inline void corioringForces()
	{
	
		T Fx 		= rate_.z() * velocity_.y() - rate_.y() * velocity_.z();
		T Fy 		= rate_.x() * velocity_.z() - rate_.z() * velocity_.x();
		T Fz 		= rate_.y() * velocity_.x() - rate_.x() * velocity_.y();
		
		Foam::Vector<T> corioring(Fx, Fy, Fz);
		
		Fcorio_ 	= corioring;
	}
	
	inline void aerodynamicForces()
	{
		
		T 	Fx 	=  - 0.5 * W_.density() * MDI_.cfx() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		T 	Fy 	= 0.5 * W_.density() * MDI_.cfy() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		T 	Fz 	= - 0.5 * W_.density() * MDI_.cfz() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		
		Foam::Vector<T> Faero(Fx, Fy, Fz);
		
		Faero_ 	= Faero;
	
	}
	
	inline void externalForces()
	{
		
	}
	
	inline void sumForces()
	{
		Fsum_		= Faero_ + Fcorio_ + Fgra_ + Fext_;
	}
	
	inline void corioringMoments()
	{
	
		T Mx 		= (I_(1,1) - I_(2,2)) * rate_.y() * rate_.z() - I_(2,0) * rate_.x() * rate_.y();
		T My 		= (I_(2,2) - I_(0,0)) * rate_.x() * rate_.z() - I_(2,0) * (rate_.z() * rate_.z() - rate_.x() * rate_.x());
		T Mz 		= (I_(0,0) - I_(1,1)) * rate_.x() * rate_.y() + I_(2,0) * rate_.y() * rate_.z();
		
		Foam::Vector<T> corioring(Mx, My, Mz);
		
		Mcorio_ 	= corioring;
	}
	
	inline void aerodynamicMoments()
	{
		
		T 	Mx 	= 0.5 * W_.density() * MDI_.cmx() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		T 	My 	= 0.5 * W_.density() * MDI_.cmy() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		T 	Mz 	= 0.5 * W_.density() * MDI_.cmz() * mach_ * mach_ * W_.soundSpeed() * W_.soundSpeed();
		
		Foam::Vector<T> Maero(Mx, My, Mz);
		
		Maero_ 	= Maero;
		
	}
	
	inline void externalMoments()
	{
		
	}
	
	inline void sumMoments()
	{
		Msum_		= Maero_ + Mcorio_ + Mext_;
	}
	
	inline void bodyToNED()
	{
        
		Foam::Tensor<T> bToN	(Foam::cos(attitude_.y()) * Foam::cos(attitude_.z()),
					 Foam::sin(attitude_.x()) * Foam::sin(attitude_.y()) * Foam::cos(attitude_.z()) - Foam::cos(attitude_.x()) * Foam::sin(attitude_.z()),
					 Foam::cos(attitude_.x()) * Foam::sin(attitude_.y()) * Foam::cos(attitude_.z()) + Foam::sin(attitude_.x()) * Foam::sin(attitude_.z()),
					 Foam::cos(attitude_.y()) * Foam::sin(attitude_.z()),
					 Foam::sin(attitude_.x()) * Foam::sin(attitude_.y()) * Foam::sin(attitude_.z()) + Foam::cos(attitude_.x()) * Foam::cos(attitude_.z()),
					 Foam::cos(attitude_.x()) * Foam::sin(attitude_.y()) * Foam::sin(attitude_.z()) + Foam::sin(attitude_.x()) * Foam::cos(attitude_.z()),
				       - Foam::sin(attitude_.y()),
					 Foam::sin(attitude_.x()) * Foam::cos(attitude_.y()),
					 Foam::cos(attitude_.x()) * Foam::cos(attitude_.y()));
        		
        	bodyToNED_	= bToN;
	}
	
	inline void bodyToEuler()
	{
	
		Foam::Tensor<T> bToE 	(1., 
					Foam::sin(attitude_.x()) * Foam::sin(attitude_.y()) / Foam::cos(attitude_.y()),
					Foam::cos(attitude_.x()) * Foam::sin(attitude_.y()) / Foam::cos(attitude_.y()),
					0.,
					Foam::cos(attitude_.x()),
				      - Foam::sin(attitude_.x()),
				      	0.,
				      	Foam::sin(attitude_.x()) / Foam::cos(attitude_.y()),
				      	Foam::cos(attitude_.x()) / Foam::cos(attitude_.y())
					);
					
		rateToEul_ 	= bToE;
	}
	
	inline void relativeVelocity()
	{
	
		relVel_	= velocity_ - W_.velocity();
		
	}
	
	inline void scalarMach()
	{
	
		mach_ 				= Foam::sqrt(relVel_.x() * relVel_.x() + relVel_.y() * relVel_.y() + relVel_.z() * relVel_.z()) / W_.soundSpeed();
				
	}
	
	inline void scalarAOA()
	{
	
		aoa_ 				= (Foam::atan2(W_.direction().z(), W_.direction().x()) + this -> attitude_.y()) * 180. / 3.14;
	
	}
	
	inline void scalarAOS()
	{
	
		aos_ 				= Foam::asin(W_.direction().y()) * 180. / 3.14;
	
	}
	
	inline void database()
	{
	
		MDI_.interpolate(mach_, aoa_, aos_);
		
	}
	
	void update(Foam::scalar deltaT)
	{
		SDRBM_.update(false, Fsum_, Msum_, deltaT, deltaT);
		
		velocity_ 		= SDRBM_.v();
		rate_ 			= SDRBM_.omega();
		
		T phi			= Foam::atan2(SDRBM_.orientation()(3,2), SDRBM_.orientation()(3,3));
		T theta 		= Foam::atan2(-SDRBM_.orientation()(3,1), Foam::sqrt(Foam::pow(SDRBM_.orientation()(3,2), 2.) + Foam::pow(SDRBM_.orientation()(3,3), 2.)));
		T psi 			= Foam::atan2(SDRBM_.orientation()(2,1), SDRBM_.orientation()(1,1));
		
		Foam::Vector<T> attitude(phi, theta, psi);
		
		attitude_ 		= attitude;
		
		Foam::Vector<T> position(SDRBM_.centreOfMass());
			
		this -> relativeVelocity();
		this -> scalarMach();
		this -> scalarAOA();
		this -> scalarAOS();
		
		this -> database();
		
		this -> gravitationalForces();
		this -> corioringForces();
		this -> aerodynamicForces();
		this -> externalForces();
		
		this -> sumForces();
		
		this -> corioringMoments();
		this -> aerodynamicMoments();
		this -> externalMoments();
		
		this -> sumMoments();
		
		SDRBM_.newTime();
	}
};
