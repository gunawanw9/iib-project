#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip> 
#include <fstream>
#include <stdio.h>
#include "string.h"
#include "lime/LimeSuite.h"
#include "reciever_setup.h"

using namespace std;

// g++ main.cpp reciever_setup.cpp -std=c++11 -lLimeSuite -o vctcxo.out

/* Entry Point */
int main(int argc, char** argv){

    /* Hardware Config */
    reciever_configuration config;
    config.rx_centre_frequency = 868e6;                 // RX Center Freuency    
    config.rx_antenna = LMS_PATH_LNAW;                  // RX RF Path = 10MHz - 2GHz
    config.rx_gain = 0.7;                               // RX Normalised Gain - 0 to 1.0
    config.enable_rx_LPF = true;                        // Enable RX Low Pass Filter
    config.rx_LPF_bandwidth = 10e6;                     // RX Analog Low Pass Filter Bandwidth
    config.enable_rx_cal = true;                        // Enable RX Calibration
    config.rx_cal_bandwidth = 8e6;                      // Automatic Calibration Bandwidth
    
    config.sample_rate = 30.72e6;                       // Sample Rate 
    config.rf_oversample_ratio = 4;                     // ADC Oversample Ratio
    
    configure_reciever(config);
    
    /* RX Stream Config  */
    lms_stream_t rx_stream;
    rx_stream.channel = 0;                              // Channel Number
    rx_stream.fifoSize = 1360 * 4096;                   // Fifo Size in Samples
    rx_stream.throughputVsLatency = 1.0;                // Optimize Throughput (1.0) or Latency (0)
    rx_stream.isTx = false;                             // TX/RX Channel
    rx_stream.dataFmt = lms_stream_t::LMS_FMT_I12;      // Data Format - 12-bit sample stored as int16_t
    LMS_SetupStream(device, &rx_stream);

    /* RX Data Buffer */
    const int num_rx_samples = 1360;
    const int rx_buffer_size = num_rx_samples * 2;
    int16_t rx_buffer[rx_buffer_size];
    
    /* RX Stream Metadata */
    lms_stream_meta_t rx_metadata;

    /* Book Keeping Indicies */
    int i = 0;
    uint64_t freq = 0;
    uint64_t pps_sync_idx = 0;
    uint64_t prev_pps_sync_idx = 0;

    /* VCTCXO DAC Controller - 0 to 4095 -> 0v to 2.538v */
    uint16_t dac_value = 3229;
    uint16_t tmp_value = 0;
    
    double target_freq = 30720000;
    double freq_err = 0.25; 
    bool dac_updated = false;
    
    int j = 0;
    bool first_buff = true;
    const int ma_len = 10;
    uint64_t readings[ma_len];
    double avg_freq = 0;
    

    /* DAC Init */
    LMS_VCTCXORead(device, &tmp_value);
    cout << "DAC = " << tmp_value << endl;
    cout << "Writing " << dac_value << " to DAC..." <<endl;
    LMS_VCTCXOWrite(device, dac_value);
    LMS_VCTCXORead(device, &tmp_value);
    cout << "DAC = " << tmp_value << endl;

    /* Start streaming */
    LMS_StartStream(&rx_stream);

    /* Process Stream for 300s */
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = t1;
    while (chrono::high_resolution_clock::now() - t1 < chrono::seconds(300)){

        /* Read Samples into Buffer */
        if(LMS_RecvStream(&rx_stream, rx_buffer, num_rx_samples, &rx_metadata, 1000) != num_rx_samples){
            LMS_StopStream(&rx_stream);
            LMS_DestroyStream(device, &rx_stream);
            error();
        };

        /* Check PPS Sync Flag - MSB Set */
        if((rx_metadata.timestamp & 0x8000000000000000) == 0x8000000000000000){
            
            /* Extract PPS Sync Index - Clear MSB */
            prev_pps_sync_idx = pps_sync_idx;
            pps_sync_idx = rx_metadata.timestamp ^ 0x8000000000000000;
            
            /* Test for Unique PPS Event */
            if (pps_sync_idx != prev_pps_sync_idx){
                
                
                /* Calculate Instantaneous Frequency */
                freq = pps_sync_idx - prev_pps_sync_idx;
                

                /* Check if DAC Value Changed over Last Second */
                if(dac_updated){

                    /* Zero Stats */
                    first_buff = true;
                    for (int f=0;f<ma_len;f++){
                        readings[f] = 0;
                    }
                    j=0;
                    dac_updated = false;

                } else {

                    /* Calculate Moving Average */
                    if(first_buff){
                        readings[j] = freq;
                        j++;
                        avg_freq = 0;
                        for(int k=0;k<j;k++){
                            avg_freq += readings[k];
                        }
                        avg_freq = avg_freq/j;
                        if(j==ma_len){
                            first_buff = false;
                        }
                    } else {
                        if(j==10){
                            j=0;
                        }
                        readings[j] = freq;
                        j++;
                        avg_freq = 0;
                        for(int m=0;m<ma_len;m++){
                            avg_freq += readings[m];
                        }
                        avg_freq = avg_freq/ma_len;
                    }

                    /* Display Frequency Stats */
                    cout << "Freq: " << freq << endl;
                    cout << "History: ";
                    for (int n=0;n<ma_len;n++){
                        cout << readings[n] << " ";
                    }
                    cout << endl;
                    cout << std::setprecision(12) << "Avg: " << avg_freq << endl << endl;


                    /* Reduce Frequency */
                    if(avg_freq > target_freq + freq_err){
                        
                        dac_value--;
                        LMS_VCTCXOWrite(device, dac_value);
                        LMS_VCTCXORead(device, &tmp_value);
                        cout << "DAC = " << tmp_value << endl;
                        dac_updated = true;
                    }

                    /* Increase Frequency */
                    if(avg_freq < target_freq - freq_err){
                        
                        dac_value++;
                        LMS_VCTCXOWrite(device, dac_value);
                        LMS_VCTCXORead(device, &tmp_value);
                        cout << "DAC = " << tmp_value << endl;
                        dac_updated = true;
                    }
                } 
            }
        }
    }

    /* Stop Streaming */
    LMS_StopStream(&rx_stream);
    
    /* Destroy Stream */
    LMS_DestroyStream(device, &rx_stream);

    /* Disable RX Channel */
    if (LMS_EnableChannel(device, LMS_CH_RX, 0, false)!=0)
        error();

    /* Close Device */
    LMS_Close(device);

    return 0;
}