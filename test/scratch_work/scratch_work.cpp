#include <iostream>
#include <fstream>
#include <complex>

#include <fftw3.h>

#define FAST_TIME 512
#define SLOW_TIME 64
#define RX 4
#define TX 3
#define IQ 2
#define SIZE_W_IQ TX*RX*FAST_TIME*SLOW_TIME*IQ
#define SIZE TX*RX*FAST_TIME*SLOW_TIME

void readFile(const std::string& filename, float* arr, int size) {
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        
        int i = 0;
        while (std::getline(file, line)) {
            if(i > size){
                 std::cerr << "Error: More samples than SIZE " << filename << std::endl;
                 break;
            }
            float value = std::stof(line);
            arr[i] = value;
            i++;
        }
        std::cout << "File Successfully read!" << std::endl;
        file.close();
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
}

// output indices --> {IQ, FAST_TIME, SLOW_TIME, RX, TX}
void getIndices(int index_1D, int* indices){
  int iq=2;
  
  int i0 = index_1D/(RX*IQ*FAST_TIME*TX);
  int i1 = index_1D%(RX*IQ*FAST_TIME*TX);
  int i2 = i1%(RX*IQ*FAST_TIME);
  int i3 = i2%(RX*IQ);
  int i4 = i3%(RX);
  
  indices[2] = i0;                    // SLOW_TIME | Chirp#
  indices[0] = i1/(RX*IQ*FAST_TIME);  // TX#
  indices[3] = i2/(RX*IQ);            // FAST_TIME | Range#
  indices[4] = i3/(RX);                 // IQ
  indices[1] = i4;                    // RX#
}

void shape_cube(float* in, float* mid, std::complex<float>* out) { 
  int rx=0;
  int tx=0;
  int iq=0;
  int fast_time=0;
  int slow_time=0;
  int indices[5] = {0};
  
  for (int i =0; i<SIZE_W_IQ; i++) {
    getIndices(i, indices);
    tx=indices[0]*RX*SLOW_TIME*FAST_TIME*IQ;
    rx=indices[1]*SLOW_TIME*FAST_TIME*IQ;
    slow_time=indices[2]*FAST_TIME*IQ;
    fast_time=indices[3]*IQ;
    iq=indices[4];
    mid[tx+rx+slow_time+fast_time+iq]=in[i];
  }

  for(int i=0; i<SIZE; i++){
    out[i]=std::complex<float>(mid[2*i+0], mid[2*i+1]);
  }
}

int compute_range_doppler(std::complex<float>* adc, std::complex<float>* rdm) {
  const int rank = 2;
  const int n[] = {SLOW_TIME, FAST_TIME};
  const int howmany = TX*RX;
  const int idist = SLOW_TIME*FAST_TIME;
  const int odist = SLOW_TIME*FAST_TIME;
  const int istride = 1;
  const int ostride = 1;

  fftwf_plan plan = fftwf_plan_many_dft(rank, n, howmany,
					reinterpret_cast<fftwf_complex*>(adc), n, istride, idist,
					reinterpret_cast<fftwf_complex*>(rdm), n, ostride, odist,
					FFTW_FORWARD, FFTW_ESTIMATE);

  fftwf_execute(plan);
  return 0;
}


int main() {
    // Create a 1D float array
  auto adc_data_flat = reinterpret_cast<float*>(malloc(SIZE_W_IQ*sizeof(float)));
  auto adc_data_reshaped = reinterpret_cast<float*>(malloc(SIZE_W_IQ*sizeof(float)));
  auto rdm_data = reinterpret_cast<std::complex<float>*>(malloc(SIZE * sizeof(std::complex<float>)));
  std::complex<float> *adc_data;  
  readFile("../data/adc_data/adc_data00.txt", adc_data_flat, SIZE_W_IQ);
  adc_data=reinterpret_cast<std::complex<float>*>(adc_data_flat);
  shape_cube(adc_data_flat, adc_data_reshaped, adc_data);
  compute_range_doppler(adc_data, rdm_data);
  
  std::cout << adc_data[0] << std::endl;
    
  return 0;
}
