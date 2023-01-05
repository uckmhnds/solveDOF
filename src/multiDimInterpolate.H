#include "fvCFD.H"
#include "IFstream.H"

// Copy an initializer list into a Matrix.
template<class Form, class Type>
void assignMatrix
(
    Matrix<Form, Type>& A,
    std::initializer_list<typename Matrix<Form, Type>::cmptType> list
)
{
    const label nargs = list.size();

    if (nargs != A.size())
    {
        FatalErrorInFunction
            << "Mismatch in matrix dimension ("
            << A.m() << ", "
            << A.n() << ") and number of args (" << nargs << ')' << nl
            << exit(FatalError);
     }

    std::copy(list.begin(), list.end(), A.begin());
}

template<typename T>
class multiDimInterpolate
{
private:

	// Matrix of look up table and its transpose
	// matrixt_ is database itself
	// matrix_ is transpose
	Foam::RectangularMatrix<T> matrix_, matrixt_;

	Foam::SquareMatrix<T> beta1;
	Foam::RectangularMatrix<T> Q1;
	Foam::RectangularMatrix<T> result_;
	
	// Name of look up table file
	Foam::word filename_;
	
	//
	T delX, delY, delZ;
	
	// A pointer to access database values
	T* ptr_;
	
	// ULists of individual database
	Foam::UList<T> Mach_, AoA_, AoS_, cfx_, cfy_, cfz_, cmx_, cmy_, cmz_;
	
	// Lower band values
	T machLower, aoaLower, aosLower;
	
	// Upper band values
	T machUpper, aoaUpper, aosUpper;
	
	// Values to interpolate on
	T machMid, aoaMid, aosMid;
	
	// Corner values (p000, p001, p010, p011, p100, p101, p110, p111)
	
	// cfx
	T cfx000, cfx001, cfx010, cfx011, cfx100, cfx101, cfx110, cfx111;
	
	// cfy
	T cfy000, cfy001, cfy010, cfy011, cfy100, cfy101, cfy110, cfy111;
	
	// cfz
	T cfz000, cfz001, cfz010, cfz011, cfz100, cfz101, cfz110, cfz111;
	
	// cmx
	T cmx000, cmx001, cmx010, cmx011, cmx100, cmx101, cmx110, cmx111;
	
	// cmy
	T cmy000, cmy001, cmy010, cmy011, cmy100, cmy101, cmy110, cmy111;
	
	// cmz
	T cmz000, cmz001, cmz010, cmz011, cmz100, cmz101, cmz110, cmz111;	

public:
	
	// Constructors
		
		// Null constructor
		multiDimInterpolate(){};
		/*
		// Copy constructor
		multiDimInterpolate
		(
			const multiDimInterpolate<T>& mdi
		)
		{

		};*/
		// Construct with a dictionary
		multiDimInterpolate
		(
			Foam::dictionary dict
		):
		filename_(dict.get<word>("databaseFile"))
		{
			
			// Create Istream with given input filename (see -> explicit IFstream(const fileName& pathname, IOstreamOption streamOpt = IOstreamOption()) in IFstream.H)
			Foam::IFstream file__(filename_);
			
			// Construct matrix from Istream (see -> explicit Matrix(Istream& is) in Matrix.H)
			Foam::RectangularMatrix<T> matrix__(file__);
			
			// Copy the database (see -> Matrix(const Matrix<Form, Type>& mat) in Matrix.H)
			matrixt_	= matrix__;
			
			// Transpose and copy the database (see -> Form T() const / Matrix(const Matrix<Form, Type>& mat) in Matrix.H)
			matrix_		= matrix__.T();
			
			// Create ULists
			createULists();
			
			
			// Create corner values
			corner_values();
			
			// DEBUG
			//Foam::Info << access_aoa() << Foam::endl;
					
		};
		
		// Construct with a matrix (Foam::Matrix)
		multiDimInterpolate
		(
			Foam::RectangularMatrix<T> matrix
		):
		matrix_(matrix)
		{
					
		};

		// Construct with input file name (Foam::word)
		multiDimInterpolate
		(
			Foam::word filename
		):
		filename_(filename)
		{
			
			// Create Istream with given input filename (see -> explicit IFstream(const fileName& pathname, IOstreamOption streamOpt = IOstreamOption()) in IFstream.H)
			Foam::IFstream file__(filename_);
			
			// Construct matrix from Istream (see -> explicit Matrix(Istream& is) in Matrix.H)
			Foam::RectangularMatrix<T> matrix__(file__);
			
			// Copy the database (see -> Matrix(const Matrix<Form, Type>& mat) in Matrix.H)
			matrixt_	= matrix__;
			
			// Transpose and copy the database (see -> Form T() const / Matrix(const Matrix<Form, Type>& mat) in Matrix.H)
			matrix_		= matrix__.T();
			
			// Create ULists
			createULists();
			
			
			// Create corner values
			corner_values();
			
			// DEBUG
			//Foam::Info << access_aoa() << Foam::endl;
			
		};
		
			
	// Destructor
		~multiDimInterpolate(){};
		
		
	// Member functions
	
		// Access
			
			// All database
			
			inline Foam::RectangularMatrix<T> access_all()
			{
				return matrix_;
			}
						
			// 
			inline T cfx()
			{
				return result_(0,0);
			}
			
			// 
			inline T cfy()
			{
				return result_(0,1);
			}
			
			// 
			inline T cfz()
			{
				return result_(0,2);
			}
						
			// 
			inline T cmx()
			{
				return result_(0,3);
			}
			
			// 
			inline T cmy()
			{
				return result_(0,4);
			}
			
			// 
			inline T cmz()
			{
				return result_(0,5);
			}
			
		// Operations
		
			// Create individual ULists from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
			// Accessing each columns first val memory address via a T*
			
			void createULists()
			{
				// Pointer at the beginning of matrix_ (see -> inline iterator begin() in Matrix.H)
				T * ptr_
				(
					matrix_.begin()				// Address of first Mach
				);
				
				// Create Mach UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> Mach
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member Mach_ (see -> inline void swap(UList<T>& list) in UList.H)
				Mach_.swap(Mach);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create AoA UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> AoA
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member AoA_ (see -> inline void swap(UList<T>& list) in UList.H)
				AoA_.swap(AoA);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create AoS UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)	
				Foam::UList<T> AoS
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member AoS_ (see -> inline void swap(UList<T>& list) in UList.H)
				AoS_.swap(AoS);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cfx UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cfx
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cfx_ (see -> inline void swap(UList<T>& list) in UList.H)
				cfx_.swap(cfx);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cfy UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cfy
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cfy_ (see -> inline void swap(UList<T>& list) in UList.H)
				cfy_.swap(cfy);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cfz UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cfz
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cfz_ (see -> inline void swap(UList<T>& list) in UList.H)
				cfz_.swap(cfz);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cmx UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cmx
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cmx_ (see -> inline void swap(UList<T>& list) in UList.H)
				cmx_.swap(cmx);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cmy UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cmy
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cmy_ (see -> inline void swap(UList<T>& list) in UList.H)
				cmy_.swap(cmy);
				
				// Move pointer to next column
				ptr_ 		+= matrix_.n();
				
				// Create cmz UList from database (see -> inline UList(T* __restrict__ v, label size) noexcept in UList.H)
				Foam::UList<T> cmz
				(
					ptr_, matrix_.n()
				);
				
				// Swap to the private member cmz_ (see -> inline void swap(UList<T>& list) in UList.H)
				cmz_.swap(cmz);
				
			}
			
			// Assign lower band values
			// Access them via an iterator (see -> inline iterator begin() / inline iterator end() in UList.H)
			
			void lower_band(T mach, T aoa, T aos)
			{
				
				for (auto iterator = Mach_.begin(); iterator < Mach_.end(); ++iterator)
				{
					if (*iterator <= mach)	{machLower = *iterator;}
					else {break;}
				}
				
				for (auto iterator = AoA_.begin(); iterator < AoA_.end(); ++iterator)
				{
					if (*iterator <= aoa)	{aoaLower = *iterator;}
					else {break;}
				}
				
				for (auto iterator = AoS_.begin(); iterator < AoS_.end(); ++iterator)
				{
					if (*iterator <= aos)	{aosLower = *iterator;}
					else {break;}
				}
				
			}
			
			// Assign upper band values
			// Access them via an iterator (see -> inline reverse_iterator rbegin() / inline reverse_iterator rend() in UList.H)
			
			void upper_band(T mach, T aoa, T aos)
			{
				
				for (auto iterator = Mach_.rbegin(); iterator < Mach_.rend(); ++iterator)
				{
					if (*iterator > mach)	{machUpper = *iterator;}
					else {break;}
				}
				
				for (auto iterator = AoA_.rbegin(); iterator < AoA_.rend(); ++iterator)
				{
					if (*iterator > aoa)	{aoaUpper = *iterator;}
					else {break;}
				}
				
				for (auto iterator = AoS_.rbegin(); iterator < AoS_.rend(); ++iterator)
				{
					if (*iterator > aos)	{aosUpper = *iterator;}
					else {break;}
				}
				
			}
			
			// Assign corner values (p000, p001, p010, p011, p100, p101, p110, p111)
			
			void corner_values()
			{
				corner_value_p000();
				corner_value_p001();
				corner_value_p010();
				corner_value_p011();
				corner_value_p100();
				corner_value_p101();
				corner_value_p110();
				corner_value_p111();
			}
			
			// p000
			void corner_value_p000()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machLower && matrixt_(iterRow,1) == aoaLower && matrixt_(iterRow,2) == aosLower)
					{
						cfx000		=  matrixt_(iterRow,3);
						cfy000		=  matrixt_(iterRow,4);
						cfz000		=  matrixt_(iterRow,5);
						
						cmx000		=  matrixt_(iterRow,6);
						cmy000		=  matrixt_(iterRow,7);
						cmz000		=  matrixt_(iterRow,8);
						
						Foam::Info << cfx000 << " " << cfy000 << " " << cfz000 << Foam::endl;
					}
				}
			};
			
			// p001
			void corner_value_p001()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machLower && matrixt_(iterRow,1) == aoaLower && matrixt_(iterRow,2) == aosUpper)
					{
						cfx001		=  matrixt_(iterRow,3);
						cfy001		=  matrixt_(iterRow,4);
						cfz001		=  matrixt_(iterRow,5);
						
						cmx001		=  matrixt_(iterRow,6);
						cmy001		=  matrixt_(iterRow,7);
						cmz001		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p010
			void corner_value_p010()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machLower && matrixt_(iterRow,1) == aoaUpper && matrixt_(iterRow,2) == aosLower)
					{
						cfx010		=  matrixt_(iterRow,3);
						cfy010		=  matrixt_(iterRow,4);
						cfz010		=  matrixt_(iterRow,5);
						
						cmx010		=  matrixt_(iterRow,6);
						cmy010		=  matrixt_(iterRow,7);
						cmz010		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p011
			void corner_value_p011()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machLower && matrixt_(iterRow,1) == aoaUpper && matrixt_(iterRow,2) == aosUpper)
					{
						cfx011		=  matrixt_(iterRow,3);
						cfy011		=  matrixt_(iterRow,4);
						cfz011		=  matrixt_(iterRow,5);
						
						cmx011		=  matrixt_(iterRow,6);
						cmy011		=  matrixt_(iterRow,7);
						cmz011		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p100
			void corner_value_p100()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machUpper && matrixt_(iterRow,1) == aoaLower && matrixt_(iterRow,2) == aosLower)
					{
						cfx100		=  matrixt_(iterRow,3);
						cfy100		=  matrixt_(iterRow,4);
						cfz100		=  matrixt_(iterRow,5);
						
						cmx100		=  matrixt_(iterRow,6);
						cmy100		=  matrixt_(iterRow,7);
						cmz100		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p101
			void corner_value_p101()
			{
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machUpper && matrixt_(iterRow,1) == aoaLower && matrixt_(iterRow,2) == aosUpper)
					{
						cfx101		=  matrixt_(iterRow,3);
						cfy101		=  matrixt_(iterRow,4);
						cfz101		=  matrixt_(iterRow,5);
						
						cmx101		=  matrixt_(iterRow,6);
						cmy101		=  matrixt_(iterRow,7);
						cmz101		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p110
			void corner_value_p110()
			{				
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machUpper && matrixt_(iterRow,1) == aoaUpper && matrixt_(iterRow,2) == aosLower)
					{
						cfx110		=  matrixt_(iterRow,3);
						cfy110		=  matrixt_(iterRow,4);
						cfz110		=  matrixt_(iterRow,5);
						
						cmx110		=  matrixt_(iterRow,6);
						cmy110		=  matrixt_(iterRow,7);
						cmz110		=  matrixt_(iterRow,8);
					}
				}
			};
			
			// p111
			void corner_value_p111()
			{				
				for (Foam::label iterRow = 0; iterRow < matrixt_.m(); ++iterRow)
				{
					if (matrixt_(iterRow,0) == machUpper && matrixt_(iterRow,1) == aoaUpper && matrixt_(iterRow,2) == aosUpper)
					{
						cfx111		=  matrixt_(iterRow,3);
						cfy111		=  matrixt_(iterRow,4);
						cfz111		=  matrixt_(iterRow,5);
						
						cmx111		=  matrixt_(iterRow,6);
						cmy111		=  matrixt_(iterRow,7);
						cmz111		=  matrixt_(iterRow,8);
					}
				}
			};
			
			void interpolate(T mach, T aoa, T aos)
			{
				
				lower_band(mach, aoa, aos);
				
				upper_band(mach, aoa, aos);
				
				corner_values();
								
				delX 	= (mach - machLower)/  (machUpper - machLower);
				delY 	= (aoa - aoaLower)  /  (aoaUpper - aoaLower);
				delZ 	= (aos - aosLower)  /  (aosUpper - aosLower);
				
				// Create a squareMatrix of sz size with zeros (see -> inline SquareMatrix(const label n, const zero) in SquareMatrix.H)	

				Foam::SquareMatrix<T> beta1(labelPair(8, 8), Zero);				
								
				assignMatrix
				(
					beta1,
					{
						T(1), T(0), T(0), T(0), T(0), T(0), T(0), T(0),
						T(-1), T(0), T(0), T(0), T(1), T(0), T(0), T(0),
						T(-1), T(0), T(1), T(0), T(0), T(0), T(0), T(0),
						T(-1), T(1), T(0), T(0), T(0), T(0), T(0), T(0),
						T(1), T(0), T(-1), T(0), T(-1), T(0), T(0), T(0),
						T(1), T(-1), T(-1), T(1), T(0), T(0), T(0), T(0),
						T(1), T(-1), T(0), T(0), T(-1), T(1), T(0), T(0),
						T(-1), T(1), T(1), T(-1), T(1), T(-1), T(-1), T(1),
					}
				);
				
				// Create Q1 Matrix
				Foam::RectangularMatrix<T> Q1(labelPair(1, 8), Zero);
				
				assignMatrix
				(
					Q1,
					{
						T(1), delX, delY, delZ, delX * delY, delY * delZ, delZ * delX, delX * delY * delZ
					}
				);
				
				Foam::RectangularMatrix<T> P(labelPair(8, 6), Zero);
				assignMatrix
				(
					P,
					{
						cfx000, cfy000, cfz000, cmx000, cmy000, cmz000,
						cfx001, cfy001, cfz001, cmx001, cmy001, cmz001,
						cfx010, cfy010, cfz010, cmx010, cmy010, cmz010,
						cfx011, cfy011, cfz011, cmx011, cmy011, cmz011,
						cfx100, cfy100, cfz100, cmx100, cmy100, cmz100,
						cfx101, cfy101, cfz101, cmx101, cmy101, cmz101,
						cfx110, cfy110, cfz110, cmx110, cmy110, cmz110,
						cfx111, cfy111, cfz111, cmx111, cmy111, cmz111
					}
				);
				
				Foam::RectangularMatrix<T> C (beta1 * P);
				Foam::RectangularMatrix<T> result (Q1 * C);
				
				result_ 		= result;

			}
			
};
