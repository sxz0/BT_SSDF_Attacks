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

  while (mRunning) {

    if (mQueueIn && mQueueIn->try_dequeue(segment)) {

      mFFTBatch.push_back(segment);

      if (mFFTBatch.size() == fft_batch_len) { // fft_batch_len {

        // Wait for FFT_batch segments to prepare data and send to the CPU
        ComputeFFT(mFFTBatch);

        for (unsigned int i = 0; i < mFFTBatch.size(); i++)
          mQueueOut->enqueue(mFFTBatch[i]);

        mFFTBatch.clear();
      }

    } else
      usleep(1);
  }
}

void FFT::ComputeFFT(std::vector<SpectrumSegment *> &segments) {

//  std::uniform_real_distribution<double> dist(0, 10);
//  std::random_device urandom("/dev/urandom");

//  bool exchange = false;
//  bool copy_1freq = true;
//  bool copy_2freq= true;
//  int count = 0;

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


//--------------------------
// NOISE
//-------------------------
/*
    if (segments[i]->getCenterFrequency() > 100000000 && segments[i]->getCenterFrequency() < 102000000){ 
      for (unsigned int j=0; j < 3; j++){
	float randomValue = dist(urandom);
        std::complex<float> z1 = randomValue * 1i;
	signal_freq[j] = signal_freq[j] + z1;
      }
    }
*/
//-------------


//--------------------------
//MIMIC
//-------------------------
/*
      std::string data;
      std::complex<float> f;

      if (segments[i]->getCenterFrequency() > 90000000 && segments[i]->getCenterFrequency() < 91000000 ) {
        //Create folders if they do not exists
        std::string root_folder_name ="/root/mimic/";
        mkdir(root_folder_name.c_str(), 0777);
        std::ofstream file_write_1 ("/root/mimic/vector.txt");

        for(unsigned int j = 0; j < 3; j++){
          file_write_1 << signal_freq[j] << std::endl;
        }
        file_write_1.close();
      }
      else if (segments[i]->getCenterFrequency() > 300000000 && segments[i]->getCenterFrequency() < 301000000) {
        int counter =0;
        // Copy in  Iq_vector the content of file 2
        std::ifstream file_2 ("/root/mimic/vector.txt");
        while (!file_2.eof()) {
          file_2 >> data;
          std::istringstream is(data);
          is>>f;
          signal_freq[counter] = f;
          counter = counter + 1;
        }
        file_2.close();
      }

*/
//------------------------


//-----------------------------
// REPEAT
//-----------------------
/*

      std::string data;
      std::complex<float> f;


      if (segments[i]->getCenterFrequency()  >= 400000000 && segments[i]->getCenterFrequency()  <= 401000000) {
        //Create folders if they do not exists
        std::string root_folder_name ="/root/repeat/";
        mkdir(root_folder_name.c_str(), 0777);

        std::ifstream file("/root/repeat/vector.txt");

        if (file.good()) {
	int counter =0;
         while (!file.eof()) {
           file >> data;
           std::istringstream is(data);
           is>>f;
           signal_freq[counter] = f;
           counter = counter + 1;
         }
          file.close();
        }
        else {
          std::ofstream outfile ("/root/repeat/vector.txt");
          for(unsigned int i = 0; i < 25; i++) {
            outfile << signal_freq[i] << std::endl;
          }
          outfile.close();
        }
     }

*/
//-------------------------


//--------------------------
// SPOOF
//-------------------------
/*
      std::string data;
      std::complex<float> f;

      if (segments[i]->getCenterFrequency() > 90000000 && segments[i]->getCenterFrequency() < 91000000 ) {
        //Create folders if they do not exists
        std::string root_folder_name ="/root/spoof/";
        mkdir(root_folder_name.c_str(), 0777);
        std::ofstream file_write_1 ("/root/spoof/vector.txt");

        for(unsigned int j = 0; j < 3; j++){
          file_write_1 << signal_freq[j] << std::endl;
        }
        file_write_1.close();
      }
      else if (segments[i]->getCenterFrequency() > 300000000 && segments[i]->getCenterFrequency() < 301000000) {
        int counter =0;
        // Copy in  Iq_vector the content of file 2
        std::ifstream file_2 ("/root/spoof/vector.txt");
        while (!file_2.eof()) {
          file_2 >> data;
          std::istringstream is(data);
          is>>f;
          float randomValue = dist(urandom);
          std::complex<float> z1 = randomValue * 1i;
          signal_freq[counter] = f + z1;
          counter = counter + 1;
        }
        file_2.close();
      }

*/
//----------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------
// CONFUSION
//--------------------
/*

      std::string data;
      std::complex<float> f;
      //std::vector<std::complex<float>> aux;

      if (!exchange && segments[i]->getCenterFrequency() > 300000000 && segments[i]->getCenterFrequency() < 301000000) {
         //Create folders if they do not exists
         std::string root_folder_name ="/root/disorder/";
         mkdir(root_folder_name.c_str(), 0777);

         std::ofstream file_write_1 ("/root/disorder/copy_vector_1.txt");
         std::ofstream file_write_2 ("/root/disorder/copy_vector_2.txt");
         for(unsigned int i = 0; i < 25; i++){
           file_write_1 << signal_freq[i] << std::endl;
           file_write_2 << signal_freq[i] << std::endl;
         }
         file_write_1.close();
         file_write_2.close();
         exchange = true;
      }
      else{
        if (exchange && (segments[i]->getCenterFrequency() > 90000000 && segments[i]->getCenterFrequency() < 91000000)){
          if (copy_1freq){

            // Copy in file 1 the content of Iq_vector
            std::ofstream file_write_1 ("/root/disorder/copy_vector_1.txt");
            for(unsigned int i = 0; i < 25; i++){
              file_write_1 << signal_freq[i] << std::endl;
            }
            file_write_1.close();

            // Copy in  Iq_vector the content of file 2
            std::ifstream file_2 ("/root/disorder/copy_vector_2.txt");

            int counter =0;
            while (!file_2.eof()) {
              file_2 >> data;
              std::istringstream is(data);
              is>>f;
              signal_freq[counter] = f;
              counter = counter + 1;
            }
            file_2.close();
            copy_1freq = false;

          }else{
            // Copy in  Iq_vector the content of file 2
            std::ifstream file_2 ("/root/disorder/copy_vector_2.txt");

            int counter =0;
            while (!file_2.eof()) {
              file_2 >> data;
              std::istringstream is(data);
              is>>f;
              signal_freq[counter] = f;
              counter = counter + 1;
            }
            file_2.close();
            copy_2freq = true;
          }
        }
        if (exchange && (segments[i]->getCenterFrequency() > 300000000 && segments[i]->getCenterFrequency() < 301000000)){
          if (copy_2freq){
            // Copy in file 2 the content of iq_vector
            std::ofstream file_write_2 ("/root/disorder/copy_vector_2.txt");
            for(unsigned int i = 0; i < 25; i++){
              file_write_2 << signal_freq[i] << std::endl;
            }
            file_write_2.close();

            // Copy in iq_vector the content of file 1
            std::ifstream file_1 ("/root/disorder/copy_vector_1.txt");

            int counter = 0;
            while (!file_1.eof()) {
              file_1 >> data;
              std::istringstream is(data);
              is>>f;
              signal_freq[counter] = f;
              counter = counter + 1;
            }
            file_1.close();
            copy_2freq = false;
          }else{
            //Copy in iq_vector the content of file 1
            std::ifstream file_1 ("/root/disorder/copy_vector_1.txt");

            int counter =0;
            while (!file_1.eof()) {
              file_1 >> data;
              std::istringstream is(data);
              is>>f;
              signal_freq[counter] = f;
              counter = counter + 1;
            }
            file_1.close();
            copy_1freq = true;
          }
        }
      }

*/
//-----------------------------------------------

//-------------------------------------------------------------------------------------------------------------
// DELAY -
//-------------------------------------------------------------------
/*

      if(segments[i]->getCenterFrequency() > 80000000 && segments[i]->getCenterFrequency() < 81000000){
        //Create folders if they do not exists
        std::string root_folder_name ="/root/delay/";
        mkdir(root_folder_name.c_str(), 0777);
        std::string segment_folder_name ="/root/delay/"+std::to_string(segments[i]->getCenterFrequency());
        const char* dirname = segment_folder_name.c_str();
        mkdir(dirname, 0777);

        //Generate the name of the files saving the PSD of each RF Segment
        count = 0;
        std::string filename ="/root/delay/"+std::to_string(segments[i]->getCenterFrequency())+"/"+std::to_string(current_time.tv_sec)+"_"+std::to_string(count)+".txt";
        std::ifstream f (filename);
        bool exist = f.good();

        while (exist){
          count = count + 1;
          filename ="/root/delay/"+std::to_string(segments[i]->getCenterFrequency())+"/"+std::to_string(current_time.tv_sec)+"_"+std::to_string(count)+".txt";
          std::ifstream f (filename);
          exist = f.good();
        }

        //Creates the file and saves the PSD values of the segment
        std::ofstream file (filename);
        for(unsigned int i = 0; i <26; i++) {
          file << signal_freq[i] << std::endl;
        }
        file.close();
        //std::cout <<"+Create new file :" <<filename << std::endl;

        //Copies the PSD values of the oldest file in the iq_vector and delets the files
        int time = current_time.tv_sec - 0000000100;
        std::string str;
        std::vector<std::string> files;
        int num;
        DIR *dr;
        struct dirent *en;

        dr = opendir(segment_folder_name.c_str()); //open all directory
        if (dr) {
          while ((en = readdir(dr)) != NULL) {
            str = en->d_name;
            //std::cout<<str<<" ";
            if (str != "." && str != ".."){
                files.push_back(str);
            }
          }
          std::sort(files.begin(), files.end());
          for (const std::string &s_aux:files){
              str = s_aux;
              //std::cout << "str: " << str << std::endl;
              str = str.substr(0,10);
              std::stringstream ss;
              ss << str;
              ss >> num;
              //std::string var (en->d_name);
              //std::cout<<str<<" ";
              //std::cout<<num<<" "<<std::endl;
              //std::cout <<"file Name: " << var << " File time: " << num << " time to delete: "<< time <<std::endl;
              if ((num < time)){
                //std::cout << "Old File found: " << en->d_name << std::endl;
                filename = "/root/delay/"+std::to_string(segments[i]->getCenterFrequency())+"/"+s_aux;
                //std::cout <<"=Copy from Old file to vector :" <<filename << std::endl;
                std::ifstream file (filename);
                //std::cout<<filename<<std::endl;
                if (file.good()) {

		  int iterator = 0;
                  while (!file.eof()) {
                    std::complex<float> f_f;
                    std::string data;
                    file >> data;
                    std::istringstream is(data);
                    is >> f_f;
                    signal_freq[iterator] = f_f;
                    iterator = iterator + 1;
                  }
                  file.close();
                  //std::cout << "Delete: " << filename <<std::endl;
                  remove(filename.c_str());
                  break;
                }
              }
          }
          closedir(dr); //close all directory
        }
      }

*/
//--------------------------------------------------------------------------------------------------------



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
