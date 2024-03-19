#include <cstdint>  // Use this for C++ (C++11 and above)
#include <vector>
#include <iostream>

#define NAVICO_SPOKE_LEN 1024

enum LookupSpokeEnum {
  LOOKUP_SPOKE_LOW_NORMAL,
  LOOKUP_SPOKE_LOW_BOTH,
  LOOKUP_SPOKE_LOW_APPROACHING,
  LOOKUP_SPOKE_HIGH_NORMAL,
  LOOKUP_SPOKE_HIGH_BOTH,
  LOOKUP_SPOKE_HIGH_APPROACHING
};

static uint8_t lookupData[6][256];

// Make space for BLOB_HISTORY_COLORS
static const uint8_t lookupNibbleToByte[16] = {
    0,     // 0
    0x32,  // 1
    0x40,  // 2
    0x4e,  // 3
    0x5c,  // 4
    0x6a,  // 5
    0x78,  // 6
    0x86,  // 7
    0x94,  // 8
    0xa2,  // 9
    0xb0,  // a
    0xbe,  // b
    0xcc,  // c
    0xda,  // d
    0xe8,  // e
    0xf4,  // f
};

void InitializeLookupData() {
  if (lookupData[5][255] == 0) {
    for (int j = 0; j <= UINT8_MAX; j++) {
      uint8_t low = lookupNibbleToByte[(j & 0x0f)];
      uint8_t high = lookupNibbleToByte[(j & 0xf0) >> 4];

      lookupData[LOOKUP_SPOKE_LOW_NORMAL][j] = (uint8_t)low;
      lookupData[LOOKUP_SPOKE_HIGH_NORMAL][j] = (uint8_t)high;

      switch (low) {
        case 0xf4: // f
          lookupData[LOOKUP_SPOKE_LOW_BOTH][j] = 0xff;
          lookupData[LOOKUP_SPOKE_LOW_APPROACHING][j] = 0xff;
          break;

        case 0xe8: // e
          lookupData[LOOKUP_SPOKE_LOW_BOTH][j] = 0xfe;
          lookupData[LOOKUP_SPOKE_LOW_APPROACHING][j] = (uint8_t)low;
          break;

        default: // else
          lookupData[LOOKUP_SPOKE_LOW_BOTH][j] = (uint8_t)low;
          lookupData[LOOKUP_SPOKE_LOW_APPROACHING][j] = (uint8_t)low;
      }

      switch (high) {
        case 0xf4: // f
          lookupData[LOOKUP_SPOKE_HIGH_BOTH][j] = 0xff;
          lookupData[LOOKUP_SPOKE_HIGH_APPROACHING][j] = 0xff;
          break;

        case 0xe8: // e
          lookupData[LOOKUP_SPOKE_HIGH_BOTH][j] = 0xfe;
          lookupData[LOOKUP_SPOKE_HIGH_APPROACHING][j] = (uint8_t)high;
          break;

        default:
          lookupData[LOOKUP_SPOKE_HIGH_BOTH][j] = (uint8_t)high;
          lookupData[LOOKUP_SPOKE_HIGH_APPROACHING][j] = (uint8_t)high;
      }
    }
  }
}





int main(int argc, char* argv[]) {
	InitializeLookupData();

	int doppler = 0;
	// std::vector<uint8_t> data(512);


	// Assuming you want a specific size for the entire array, change 100 to whatever size you need
	const size_t ARRAY_SIZE = 512;

	// Initialize the array to all zeros
	uint8_t data[ARRAY_SIZE] = {0};

	
	

	// Initialize first 42 bytes with your hex information
	uint8_t initData[42] = {
		0xee, 0xee, 0xee, 0x00, 0x30, 0x96, 0xfc, 0xff, 0xff, 0xff, 
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
		0xbe, 0x58
	};
	// Copy the initData into the beginning of myArray
	memcpy(data, initData, sizeof(initData));  // Copy first 42 bytes



	uint8_t upscaledData[ARRAY_SIZE*2] = {0};
	uint8_t dopplerData[ARRAY_SIZE*2] = {0};

	uint8_t doppler_mode = 2;


	for (int i = 0; i <  NAVICO_SPOKE_LEN/2; i++) {
		// Each byte is split into two nibbles
		// if dopplermode is 1 f is considered approaching with highest intensity and the other values are adjusted accordingly
		// if dopplermode is 2 f is considered approaching with highest intensity and e is considered receding with highest intensity and the other values are adjusted accordingly
		uint8_t low =  data[i] & 0x0f;
		uint8_t high = (data[i] >> 4) & 0x0f;

		if (doppler_mode == 0) {
			upscaledData[2 * i] = low * 0x11; // 0x11 is 17 so that 0x01 becomes 0x11 and 0x0f becomes 0xff
			upscaledData[2 * i + 1] = high * 0x11;
			dopplerData[2 * i] = 0;
			dopplerData[2 * i + 1] = 0;
		}

		// TODO: staticcast to int

		if (doppler_mode == 1) { // only approaching
			if (low == 0x0f) { // approaching
				dopplerData[2 * i] = 1;
				upscaledData[2 * i] = 0xff;
			} else {
				dopplerData[2 * i] = 0;
				if (low == 0x0e) {
					upscaledData[2 * i] = 0xff; // so it does not become 0x10A
				} else {
					upscaledData[2 * i] = (low * 0x13);
				}
			}

			if (high == 0x0f) { // approaching
				dopplerData[2 * i + 1] = 1;
				upscaledData[2 * i + 1] = 0xff;
			} else {
				dopplerData[2 * i + 1] = 0;
				if (high == 0x0e) {
					upscaledData[2 * i + 1] = 0xff; // so it does not become 0x10A
				} else {
					upscaledData[2 * i + 1] = (high * 0x13);
				}
			}
		}

		if (doppler_mode == 2) { // both approaching and receding
			if (low == 0x0f) { // approaching
				dopplerData[2 * i] = 1;
				upscaledData[2 * i] = 0xff;
			} else if (low == 0x0e) { // receding
				dopplerData[2 * i] = 2;
				upscaledData[2 * i] = 0xff;
			} else {
				dopplerData[2 * i] = 0;
				if (low == 0x0d) {
					upscaledData[2 * i] = 0xff; // so it does not become 0x104
				} else {
					upscaledData[2 * i] = (low * 0x14);
				}
			}

			if (high == 0x0f) { // approaching
				dopplerData[2 * i + 1] = 1;
				upscaledData[2 * i + 1] = 0xff;
			} else if (high == 0x0e) { // receding
				dopplerData[2 * i + 1] = 2;
				upscaledData[2 * i + 1] = 0xff;
			} else {
				dopplerData[2 * i + 1] = 0;
				if (high == 0x0d) {
					upscaledData[2 * i + 1] = 0xff; // so it does not become 0x104
				} else {
					upscaledData[2 * i + 1] = (high * 0x14);
				}
			}
		}

	}

	for (int i = 0; i < 40 ; i++) {
		std::cout << std::hex << (int)(upscaledData[i] & 0xff) << " ";
		if (i % 16 == 15) {
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;
	std::cout << std::endl;

	for (int i = 0; i < 40 ; i++) {
		std::cout << std::hex << (int)(dopplerData[i] & 0x0f) << " ";
		if (i % 16 == 5) {
			std::cout << std::endl;
		}
	}
		
	





	

	// iterate doppler modes:
	for (int d=0; d<3;d++){
		uint8_t data_highres[NAVICO_SPOKE_LEN];
		doppler = d;
		std::cout << "Doppler mode:" << doppler << std::endl;
		// value 0-256
		// uint8_t v = 255;
		// for (int s=0; s<512;s++){
		// 	data[s] = v;
		// }

		uint8_t *lookup_low = lookupData[LOOKUP_SPOKE_LOW_NORMAL + doppler];
		uint8_t *lookup_high = lookupData[LOOKUP_SPOKE_HIGH_NORMAL + doppler];
		for (int i = 0; i <  NAVICO_SPOKE_LEN / 2; i++) {
			data_highres[2 * i] = lookup_low[data[i]];
			data_highres[2 * i + 1] = lookup_high[data[i]];
		}

		// print the first 10 values
		for (int i=0; i<50;i++){
			std::cout << "data_highres[" << i << "]=" << (int)data_highres[i] << std::endl;
		}
	}
}