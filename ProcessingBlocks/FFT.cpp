/*
 * Copyright (C) 2018 by IMDEA Networks Institute
 *
 * This file is part of Electrosense.
 *
 * Electrosense is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Electrosense is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTL-Spec.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 	Authors: 	Roberto Calvo-Palomino <roberto.calvo@imdea.org>
 *
 */

#include "FFT.h"
#include <random>
//NEW
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <sstream>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <experimental/filesystem>
#include <dirent.h>


namespace electrosense {

FFT::FFT() { mQueueOut = new ReaderWriterQueue<SpectrumSegment *>(100); }

void FFT::run() {

  std::cout << "[*] FFT block running .... " << std::endl;

  unsigned int fft_batch_len =
      ElectrosenseContext::getInstance()->getFFTbatchlen();

  mRunning = true;
  SpectrumSegment *segment;

  if (mQueueIn == NULL || mQueueOut == NULL) {
    throw std::logic_error("Queue[IN|OUT] are NULL!");
  }


  //----------------------------------------------------------------------------------------------------------
  // REPEAT - Setup
  //---------------------
  // std::vector<std::complex<float>> repeat_source_segment;

  //----------------------------------------------------------------------------------------------------------
  // MIMIC - Setup
  //---------------------
  // int current_frequency = 0;
  // std::vector<std::complex<float>> mimic_source_segment;

  //----------------------------------------------------------------------------------------------------------
  // CONFUSION/EXCHANGE/DISORDER - Setup
  //---------------------
  // int current_frequency = 0;
  // std::vector<std::vector<std::complex<float>>> confusion_source_segment_1;
  // std::vector<std::vector<std::complex<float>>> confusion_source_segment_2;

  //----------------------------------------------------------------------------------------------------------
  // NOISE - Setup
  //---------------------
  //  std::uniform_real_distribution<double> dist(0, 10);
  //  std::random_device urandom("/dev/urandom");

  //-------------------------------------------------------------------------------

  //----------------------------------------------------------------------------------------------------------
  // SPOOF  - Setup
  //---------------------
  // std::uniform_real_distribution<double> dist(0, 20);
  // std::random_device urandom("/dev/urandom");
  // int current_frequency = 0;
  // std::vector<std::vector<std::complex<float>>>  spoof_source_segment;

  //-------------------------------------------------------------------------------

  //----------------------------------------------------------------------------------------------------------
  // FREEZE - Setup
  //---------------------
  // std::vector<std::vector<std::complex<float>>> freeze_source_segment;
  // int current_frequency = 0;
  // bool freeze = false;

  //-------------------------------------------------------------------------------

  //----------------------------------------------------------------------------------------------------------
  // DELAY - Setup
  //---------------------
  //  int affected_frequencies = 0;
  //  int current_frequency = 0;
  //  int current_iteration = 0;
  //  int delay = 10;
  //  bool init = true;
  //  bool full = false;
  //  std::vector<std::vector<std::vector<std::complex<float>>>> delay_source_segment;
  //  std::vector<std::complex<float>> tmp_iq_vector;
  //-------------------------------------------------------------------------------

  while (mRunning) {

    if (mQueueIn && mQueueIn->try_dequeue(segment)) {

      mFFTBatch.push_back(segment);

      if (mFFTBatch.size() == fft_batch_len) { // fft_batch_len {

        // Wait for FFT_batch segments to prepare data and send to the CPU

        //ComputeFFT_normal(mFFTBatch);
        //ComputeFFT_repeat(mFFTBatch, repeat_source_segment);
        ComputeFFT_mimic(mFFTBatch, current_frequency, mimic_source_segment);
        //ComputeFFT_confusion(mFFTBatch);
        //ComputeFFT_noise(mFFTBatch);
        //ComputeFFT_spoof(mFFTBatch);
        //ComputeFFT_freeze(mFFTBatch);
        //ComputeFFT_delay(mFFTBatch);

        for (unsigned int i = 0; i < mFFTBatch.size(); i++)
          mQueueOut->enqueue(mFFTBatch[i]);

        mFFTBatch.clear();
      }

    } else
      usleep(1);
  }
}


//----------------------------------------------------------------------------------------------------------
//  Normal
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_normal(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  // std::cout << "Compute FFT with current segment size being " << segments.size() << " and center freq " << segments[0]->getCenterFrequency() << std::endl;

  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------
    //  End -------------------------------------------------------------------------------------------------

    // std::cout << "Testing FFT for segment" << i << " with signal_len " << signal_len << " and signal_freq "<< signal_freq << std::endl;
    // std::cout << "Current valuesin signal_freq: ";
    // for(std::complex<float> i : signal_freq) 
    //   std::cout << i << ", " << std::endl;
    // std::cout << std::endl;

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Repeat
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_repeat(std::vector<SpectrumSegment *> &segments, std::vector<std::complex<float>> &repeat_source_segment) {

  //----------------------------------------------------------------------------------------------------------
  // GLOBAL - Setup
  //---------------------
  uint64_t attacked_freq_1 = 90000000;
  uint64_t attack_bw = 1000000;
  uint64_t attacked_freq_2 = 100000000;
  uint64_t attack_impact = 3;

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  

  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);

    //  Start -------------------------------------------------------------------------------------------------

    //-- Indicates the frequency segment affected by the attack
    if (segments[i]->getCenterFrequency()  >= attacked_freq_1 && segments[i]->getCenterFrequency()  <= attacked_freq_1 + attack_bw ) {

      //-- If the vector with the PSD values already exists, it copies its content to the segment to be sent
      if (!repeat_source_segment.empty()) {
        for(unsigned int i = 0; i < attack_impact; i++) {
          signal_freq[i] = repeat_source_segment[i];
        }
      }

      //-- If the vector is empty it creates it and saves the PSD values of the selected frequency segment
      else {
        for(unsigned int i = 0; i < attack_impact; i++) {
          repeat_source_segment.push_back(signal_freq[i]);
        }
      }
    }

    //  End -------------------------------------------------------------------------------------------------


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}


//----------------------------------------------------------------------------------------------------------
//  MIMIC - Copies one band in another band
//---------------------------------------
void FFT::ComputeFFT_mimic(std::vector<SpectrumSegment *> &segments, int &current_frequency,  std::vector<std::complex<float>> &mimic_source_segment) {
  
  //----------------------------------------------------------------------------------------------------------
  // GLOBAL - Setup
  //---------------------
  uint64_t attacked_freq_1 = 90000000;
  uint64_t attack_bw = 1000000;
  uint64_t attacked_freq_2 = 100000000;
  uint64_t attack_impact = 3;


  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];

  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);


    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);

    //  Start -------------------------------------------------------------------------------------------------

    std::cout << "mimic_source_segment is now " << mimic_source_segment.size() << " big" << std::endl;
        //--The segment that needs to be copied
        if (segments[i]->getCenterFrequency() > attacked_freq_1 && segments[i]->getCenterFrequency() < attacked_freq_1 + attack_bw ) {
          for(unsigned int j = 0; j < attack_impact; j++){
              mimic_source_segment.push_back(signal_freq[j]);
          }
        }
        //--The segment the copied PSD values are pasted into
        else if (segments[i]->getCenterFrequency() > attacked_freq_2 && segments[i]->getCenterFrequency() < attacked_freq_2 + attack_bw) {
          std::cout << "Current values in mimic_source_signal: ";
          for(std::complex<float> i : mimic_source_segment) 
            std::cout << i << ", " << std::endl;
          std::cout << std::endl;
          for(unsigned int j = 0; j < attack_impact; j++){
            std::cout << "Copied " << j+current_frequency*attack_impact << " position in mimic_source_segment" << std::endl;
            uint64_t position = j+current_frequency*attack_impact;
            signal_freq[j] = mimic_source_segment[position];
          }
          current_frequency++;
        }

        if (segments[i]->getCenterFrequency() > attacked_freq_2 + attack_bw) {
          mimic_source_segment.clear();
          current_frequency = 0;
        }

    //  End -------------------------------------------------------------------------------------------------


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Confusion
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_confusion(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------
    
      //-- The first frequency segment that needs to be exchanged
      if (segments[i]->getCenterFrequency() > attacked_freq_1 && segments[i]->getCenterFrequency() < attacked_freq_1 + attack_bw ) {
        //-- Save the sensed PSD values in temporary variable
        for(unsigned int j = 0; j < attack_impact; j++){
            confusion_source_segment_1.push_back(signal_freq[j]);
        }
        //-- overwrite the sensed PSD values with the second segment if it already exists
        if (!confusion_source_segment_2.empty()) {
          iq_vector.clear();
          for(unsigned int j = 0; j < attack_impact; j++){
              confusion_source_segment_1.push_back(signal_freq[j]);
          }
          if (i == ElectrosenseContext::getInstance()->getAvgFactor() - 1) {
            current_frequency++;
          }
        }
      }

      if (current_frequency != 0 && segments[i]->getCenterFrequency() > attacked_freq_1 + attack_bw  && segments[i]->getCenterFrequency() < attacked_freq_2 ) {
        confusion_source_segment_2.clear();
        current_frequency = 0;
      }

      //-- The second frequency segment that needs to be exchanged
      if (segments[i]->getCenterFrequency() > attacked_freq_2 && segments[i]->getCenterFrequency() < attacked_freq_2 + attack_bw) {
        //-- Save the sensed PSD values in temporary variable
        if (i == 0){
          confusion_source_segment_2.push_back(iq_vector);
        }
        //-- overwrite the sensed PSD values with the first segment
        iq_vector.clear();
        for(unsigned int i = 0; i < confusion_source_segment_1[current_frequency].size(); i++) {
          iq_vector.push_back(confusion_source_segment_1[current_frequency][i]);
        }
        if (i == ElectrosenseContext::getInstance()->getAvgFactor() - 1) {
          current_frequency++;
        }
      }

      if (current_frequency != 0 && center_freq > attacked_freq_2 + attack_bw) {
        confusion_source_segment_1.clear();
        current_frequency = 0;
      }

    //  End -------------------------------------------------------------------------------------------------

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Noise
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_noise(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------
      
    //-- Indicates the frequency segments affected by the attack
    if (segments[i]->getCenterFrequency() >= attacked_freq_1 && segments[i]->getCenterFrequency() <= attacked_freq_1 + attack_bw){

      //-- Adds random noise to j positions of the selected segments -- 3 when the attack impact is 20khz, and 25 when it is 200khz 
      for (unsigned int j=0; j < attack_impact; j++){
        float randomValue = dist(urandom);
        std::complex<float> z1 = randomValue * 1i;
        signal_freq[j] = signal_freq[j] + z1;
      }
    }

    //  End -------------------------------------------------------------------------------------------------

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Spoof
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_spoof(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------

    //--The segment that needs to be copied
    if (segments[i]->getCenterFrequency() > attacked_freq_1 && segments[i]->getCenterFrequency() < attacked_freq_1 + attack_bw ) {
      for(unsigned int j = 0; j < attack_impact; j++){
          mimic_source_segment.push_back(signal_freq[j]);
      }
    }
    //--The segment the copied PSD values are pasted into
    else if (segments[i]->getCenterFrequency() > attacked_freq_2 && segments[i]->getCenterFrequency() < attacked_freq_2 + attack_bw) {
      for(unsigned int j = 0; j < attack_impact; j++){
        float randomValue = dist(urandom);
        std::complex<float> z1 = randomValue * 1i;
        signal_freq[j] = mimic_source_segment[j] + z1;
      }
    }

    if (segments[i]->getCenterFrequency() > attacked_freq_2 + attack_bw) {
      mimic_source_segment.clear();
    }


    //  End -------------------------------------------------------------------------------------------------

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Freeze
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_freeze(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------

    //-- Indicates the frequency segment affected by the attack
    if (segments[i]->getCenterFrequency() > attacked_freq_1 && center_freq < attacked_freq_1 + attack_bw) {
        //-- Create the source array on the first iteration
        if (!freeze){
          if (i == ElectrosenseContext::getInstance()->getAvgFactor() -1){
            freeze_source_segment.push_back(iq_vector);
          }
        }else{
          iq_vector.clear();
          for(unsigned int i = 0; i < freeze_source_segment[current_frequency].size(); i++) {
            iq_vector.push_back(freeze_source_segment[current_frequency][i]);
          }
          if (i == ElectrosenseContext::getInstance()->getAvgFactor() -1) {
            current_frequency++;
          }
        }
      }

      if (!freeze  && center_freq > attacked_freq_1 + attack_bw) {
        freeze = true;
      }

      if (current_frequency != 0 && center_freq > attacked_freq_1 + attack_bw) current_frequency = 0;

    //  End -------------------------------------------------------------------------------------------------

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

//----------------------------------------------------------------------------------------------------------
//  Delay
//----------------------------------------------------------------------------------------------------------
void FFT::ComputeFFT_delay(std::vector<SpectrumSegment *> &segments) {

  unsigned int signal_len =
      1 << ElectrosenseContext::getInstance()->getLog2FftSize();
  int flags = 0;
  std::complex<float> signal_freq[signal_len];
  
  for (unsigned int i = 0; i < segments.size(); i++) {

    std::complex<float> *signal = segments[i]->getIQSamples().data();

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq,
                                  LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);
    fft_shift(signal_freq, signal_len);

    //  Start -------------------------------------------------------------------------------------------------
       //-- Indicates the frequency segment affected by the attack
       if (center_freq > attacked_freq_1 && center_freq < attacked_freq_1 + attack_bw) {
         //-- Init phase: create 3d array with frequencies x delay x values
         if (init && i == 0){
           std::vector<std::vector<std::complex<float>>> tmp_vector(delay);
           tmp_vector.insert(tmp_vector.begin(), iq_vector);
           delay_source_segment.push_back(tmp_vector);
           affected_frequencies++;
         }
         //-- Fill the source array with the current data
         if (!init && i == 0){
           //-- If the array is full, save the previous data to a temporary variable
           if (full){
             tmp_iq_vector.clear();
             for(unsigned int i = 0; i < delay_source_segment[current_frequency][current_iteration].size(); i++) {
               tmp_iq_vector.push_back(delay_source_segment[current_frequency][current_iteration][i]);
             }
             delay_source_segment[current_frequency][current_iteration].clear();
           }
           for(unsigned int i = 0; i < iq_vector.size(); i++) {
             delay_source_segment[current_frequency][current_iteration].push_back(iq_vector[i]);
           }
           if (full){
             iq_vector.clear();
             for(unsigned int i = 0; i < tmp_iq_vector.size(); i++) {
               iq_vector.push_back(tmp_iq_vector[i]);
             }
           }
         }

         if (!init && i == ElectrosenseContext::getInstance()->getAvgFactor() -1) {
           current_frequency++;
         }

          //-- when all frequencies are saved and/or modified go to next iteration
         if (i == ElectrosenseContext::getInstance()->getAvgFactor() -1 && current_frequency == affected_frequencies) {
           current_frequency = 0;
           current_iteration++;
           //-- when the defined delay is reached, start again
           if (current_iteration > delay){
             full = true;
             current_iteration = 0;
             current_frequency = 0;
           }
         }

        }

       //-- when init is done for all frequencies, change to "normal" mode
       if (init && center_freq > attacked_freq_1 + attack_bw){
         init = false;
         current_iteration++;
       }

    //  End -------------------------------------------------------------------------------------------------

    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);


      segments[i]->getIQSamplesFreq().assign(signal_freq,
                                          signal_freq + signal_len);

    fft_destroy_plan(q_f);
  }
}

int FFT::stop() {

  mRunning = false;
  waitForThread();

  return 1;
}
} // namespace electrosense
