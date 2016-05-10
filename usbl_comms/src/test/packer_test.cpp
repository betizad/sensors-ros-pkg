#include <labust/tools/packer.h>
#include <bitset>

/**
 * Initial test structure for the bitwise packer.
 * For real applications the member properties are defined in a XML.
 *
 * Name	Type 	Min		Max		Bits
 * x	double	-51		51		10
 * y	double	-51		51		10
 * z	double	0		128		7
 * ---------------------------------
 * 								27
 *
 * Required bytes for storage ceil(27/8) = 4
 */
struct Position
{
	double x;
	double y;
	double z;

	inline static int getSerializationSize(){return serialization_size;};

	void pack(boost::archive::binary_oarchive& out) const
	{
		//This is calculated during code generation
		static const double q((51+51)/1023.0);
		std::cout<<"quantization:"<<q<<std::endl;
		//Preallocated storage buffer
		static uint8_t storage[serialization_size];
		int bitpt(0);
		int bytept(0);
		uint64_t xw(0);
		//For each element generate a combo
			//Wrap it into a smallest type (signed or unsigned)
			xw = (x + 51)/q;
			coder(storage, &bitpt, &bytept, xw, 10);
			//Wrap it into a smallest type (signed or unsigned)
			xw = (y + 51)/q;
			coder(storage, &bitpt, &bytept, xw, 10);

		//Add everything into the archive
		for(int i=0; i<serialization_size; ++i) out << storage[i];
	}

	void coder(uint8_t* storage, int* bitpt, int* bytept, uint64_t xw, int bitsz) const
	{
		std::cout<<*bitpt<<std::endl;
		//Mask the upper bits
		std::cout<<std::bitset<16>(xw)<<std::endl;
		xw &= (1<<bitsz)-1;
		//Put it into the storage
		int rembits(bitsz);
		while (rembits)
		{
			int rem = 8-(*bitpt);
			if (rembits < rem) rem = rembits;
			uint8_t temp = uint8_t((xw >> (rembits - rem)) << (*bitpt));
			std::cout<<int(temp)<<":"<<rem<<":"<<rembits<<std::endl;
			storage[*bytept] |= uint8_t((xw >> (rembits - rem)) << (*bitpt));
			*bitpt += rem;
			if (*bitpt == 8)
			{
				++(*bytept);
				*bitpt = 0;
			}
			rembits -= rem;
			std::cout<<int(xw)<<":"<<rembits<<":comp:"<<(rembits << 1)<<std::endl;
			xw &= uint64_t((1 << rembits) - 1);
			std::cout<<int(xw)<<std::endl;
		}
	}

//private:
	static const int serialization_size = 4;
};

struct BitStorage
{
	enum {TOP_STORE=256};

	static const uint64_t one = 1;

	BitStorage():_storage(0,0),bytept(0),bitpt(0)
	{
		_storage.reserve(TOP_STORE);
	};

	///
	template<class ValueType>
	void put(const ValueType& data, double min, double max, uint8_t bitsz)
	{
		//Get the size mask (maximum 64 bits for data representation)
		uint64_t mask = (one << bitsz)-one;
		//Normalize data
		uint64_t xw = uint64_t(mask * (data - min)/(max-min));
		xw &= mask;

		uint8_t rembits(bitsz);
		while (rembits)
		{
			//Check if enough size is available
			if (_storage.size() <= bytept) _storage.push_back(0);

			int rem = 8-(bitpt);
			if (rembits < rem) rem = rembits;
			_storage[bytept] |= uint8_t((xw >> (rembits - rem)) << (bitpt));
			bitpt += rem;
			if (bitpt == 8)
			{
				++(bytept);
				bitpt = 0;
			}
			rembits -= rem;
			xw &= (one << rembits) - one;
		}
	}

	template<class ValueType>
	bool get(ValueType& data, double min, double max, uint8_t bitsz)
	{
		//Get the size mask (maximum 64 bits for data representation)
		uint64_t mask = (one << bitsz)-one;
		//Init data
		uint64_t xw(0);

		std::cout<<"Params:"<<(_storage.size() - bytept)*8-bitpt<<">"<<int(bitsz)<<std::endl;
		//Check if enough size is available
		if (((_storage.size() - bytept)*8-bitpt) < bitsz) return false;

		uint8_t rembits(bitsz);
		while (rembits)
		{
			int rem = 8-(bitpt);
			if (rembits < rem) rem = rembits;

			xw |= ((_storage[bytept] & (((one << rem) - one) << bitpt)) >> bitpt) << (rembits-rem);

			bitpt += rem;
			if (bitpt == 8)
			{
				++(bytept);
				bitpt = 0;
			}
			rembits -= rem;
		}

		data = ValueType((max-min)*xw/mask + min);
		return true;
	}

	///Get storage
	const std::vector<uint8_t>& storage() const {return _storage;}

	///Reset storage
	void reset(){bytept=0; bitpt=0;}

	///Clear storage
	void clear(){_storage.clear();reset();}

public:
	//The continuous byte storage
	std::vector<uint8_t> _storage;
	//The byte pointer
	uint32_t bytept;
	//The bit pointer
	uint32_t bitpt;
};

int main(int argc, char* argv[])
{
	std::vector<char> binary;
	Position a;
	a.x = 42.332;
	a.y = -31.889;

	BitStorage st;
	st.put(a.x, -51.2, 51.1, 10);
	st.put(a.y, -51.2, 51.1, 10);

	labust::tools::encodePackable(a, &binary);

	std::cout<<"Array:"<<std::hex<<int(uint8_t(binary[0]))<<", "<<int(uint8_t(binary[1]))<<", "<<int(uint8_t(binary[2]))<<std::endl;
	std::cout<<"Converted:"<<std::dec<<uint8_t(binary[0])*4+uint8_t(binary[1])<<std::endl;

	std::cout<<"State:"<<st.bytept<<":"<<st.bitpt<<std::endl;
	std::cout<<"Array:"<<std::hex<<int(uint8_t(st.storage()[0]))<<", "<<int(uint8_t(st.storage()[1]))<<", "<<int(uint8_t(st.storage()[2]))<<std::endl;
	std::cout<<"Converted:"<<std::dec<<uint8_t(st.storage()[0])*4+uint8_t(st.storage()[1])<<std::endl;

	st.reset();

	double test;
	a.x = -1000;
	a.y = -1000;

	if (!st.get(a.x, -51.2, 51.1, 10)) std::cout<<"run out off space."<<std::endl;
	if (!st.get(a.y, -51.2, 51.1, 10)) std::cout<<"run out off space."<<std::endl;;
	if (!st.get(test, -51.2, 51.1, 10)) std::cout<<"run out off space."<<std::endl;;

	std::cout<<"Decoded:"<<a.x<<","<<a.y<<std::endl;

	return 0;
}
