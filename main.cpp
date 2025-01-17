

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

#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include <signal.h>

// Workaround issue #4 , complex.h breaks openssl's RSA library
//   include RSA before any mention to complex.h (in rtlsdrDriver.h)
#include "MiscBlocks/Transmission.h"
#include "MiscBlocks/AvroSerialization.h"
#include "MiscBlocks/FileSink.h"
#include "MiscBlocks/IQSink.h"
#include "MiscBlocks/IQStream.h"

#include "context/ElectrosenseContext.h"
//#include "build/generated/version_config.h"
#include "generated/version_config.h"

#include "drivers/Component.h"
#include "drivers/rtlsdr/rtlsdrDriver.h"

#include "ProcessingBlocks/Averaging.h"
#include "ProcessingBlocks/FFT.h"
#include "ProcessingBlocks/PSDFast.h"
#include "ProcessingBlocks/RemoveDC.h"
#include "ProcessingBlocks/RemoveDCRTL.h"
#include "ProcessingBlocks/Windowing.h"

void usage(char *name) {

  fprintf(
      stderr,
      "Usage:\n"
      "  %s min_freq max_freq\n"
      "  [-h]\n"
      "  [-d <dev_index>]\n"
      "  [-c <clk_off>] [-k <clk_corr_period>]\n"
      "  [-g <gain>]\n"
      "  [-y <hopping_strategy>]\n"
      "  [-s <samp_rate>]\n"
      "  [-q <freq_overlap>]\n"
      "  [-t <monitor_time>] [-r <min_time_res>]\n"
      "  [-l <cmpr_level>]\n"
      "  [-m "
      "<hostname1>:<portnumber1>[;<bandwidth1>],...,<hostnameN>:<portnumberN>[;"
      "bandwidthN]]\n"
      "  [-n "
      "<hostname1>:<portnumber1>[;<bandwidth1>],...,<hostnameN>:<portnumberN>[;"
      "<bandwidthN>]#<ca_cert>#<cert>#<key>]\n"
      "  [-z <pipeline> ]\n"
      "  [-i <interface>]\n"
      "\n"
      "  PSD Pipeline\n"
      "    [-f <log2_fft_size>] [-b <fft_batchlen>]\n"
      "    [-a <avg_factor>] [-o <soverlap>] \n"
      "    [-w <window>]\n"
      "\n"
      "  IQ Pipeline\n"
      "    [-x <absolute_time>]\n"
      "\n"
      "Arguments:\n"
      "  min_freq               Lower frequency bound in Hz\n"
      "  max_freq               Upper frequency bound in Hz\n"
      "\n"
      "Options:\n"
      "  -h                     Show this help\n"
      "  -d <dev_index>         RTL-SDR device index [default=%i]\n"
      "  -z <pipeline>          Pipeline process: PSD | IQ [default=%s]\n"
      "  -c <clk_off>           Clock offset in PPM [default=%i]\n"
      "  -k <clk_corr_period>   Clock correction period in seconds "
      "[default=%u]\n"
      "                           i.e. perform frequency correction every "
      "'clk_corr_period'\n"
      "                           seconds\n"
      "  -g <gain>              Gain value in dB [default=%.1f]\n"
      "                           -1 for automatic gain\n"
      "  -e <bias-tee_enable>   Enable bias-tee: 0 | 1 [default=%i]\n"
      "  -y <hopping_strategy>  Hopping strategy to use [default=%s]\n"
      "                           sequential\n"
      "                           random\n"
      "                           similarity\n"
      "  -s <samp_rate>         Sampling rate in Hz [default=%u]\n"
      "  -q <freq_overlap>      Frequency overlapping factor [default=%.3f]\n"
      "                           i.e. the frequency width is reduced from "
      "'samp_rate' to\n"
      "                           (1-'freq_overlap')*'samp_rate'\n"
      "  -t <monitor_time>      Time in seconds to monitor [default=%u]\n"
      "                           0 to monitor infinitely\n"
      "  -r <min_time_res>      Minimal time resolution in seconds "
      "[default=%u]\n"
      "                           0 for no time resolution limitation\n"
      "  -w <window>            Windowing function [default=%s]\n"
      "                           hanning\n"
      "                           rectangular\n"
      "                           blackman_harris_4\n"
      "  -l <cmpr_level>        Compression level [default=%u]\n"
      "                           0 for no compression, fastest\n"
      "                           9 for highest compression, slowest\n"
      "  -m "
      "<hostname1>:<portnumber1>[;<bandwidth1>],...,<hostnameN>:<portnumberN>[;"
      "<bandwidthN>]\n"
      "                         TCP collector hosts [default=%s]\n"
      "                           Bandwidth limitation in Kb/s\n"
      "  -n <hostname>:<portnumber>#<ca_cert>#<cert>#<key>\n"
      "                         SSL/TLS collector host [default=%s]\n"
      "                           0 for no SSL/TLS collector\n"
      "                           SSL/TLS (CA) certificate and private key "
      "files\n"
      "  -i <interface>         Interface to authenticate towards collector [default=%s]\n"
      "  -p <true|false>        Set FIFO priority to the sampling thread "
      "[default=%s]\n"
      "                           true\n"
      "                           false\n"
      "  -u <filename>          Set filename output where Spectrum "
      "measurements are saved.\n"
      "\n"
      "PSD PIPELINE\n"
      "  -f <log2_fft_size>     Use FFT size of 2^'log2_fft_size' "
      "[default=%u]\n"
      "                           the resulting frequency resolution is\n"
      "                           'samp_rate'/(2^'log2_fft_size')\n"
      "  -b <fft_batchlen>      FFT batch length [default=%u]\n"
      "                           i.e. process FFTs in batches of length "
      "'fft_batchlen'\n"
      "  -a <avg_factor>        Averaging factor [default=%u]\n"
      "                           i.e. average 'avg_factor' segments\n"
      "  -o <soverlap>          Segment overlap [default=%u]\n"
      "                           i.e. number of samples per segment that "
      "overlap\n"
      "                           The time to dwell in seconds at a given "
      "frequency is given by\n"
      "                           "
      "(((1<<'log2_fft_size')-'soverlap')*'avg_factor'+'soverlap')/"
      "'samp_rate'\n"
      "IQ PIPELINE\n"
      "  -x <absolute_time>     Absolute time when receiver must start "
      "sampling\n"
      "",
      name, ElectrosenseContext::getInstance()->getDevIndex(),
      ElectrosenseContext::getInstance()->getPipeline().c_str(),
      ElectrosenseContext::getInstance()->getClkOffset(),
      ElectrosenseContext::getInstance()->getClkCorrPerior(),
      ElectrosenseContext::getInstance()->getGain(),
      ElectrosenseContext::getInstance()->getBiasTee(),
      ElectrosenseContext::getInstance()->getHoppingStrategy().c_str(),
      ElectrosenseContext::getInstance()->getSamplingRate(),
      ElectrosenseContext::getInstance()->getFreqOverlap(),
      ElectrosenseContext::getInstance()->getMonitorTime(),
      ElectrosenseContext::getInstance()->getMinTimeRes(),
      ElectrosenseContext::getInstance()->getWindowing().c_str(),
      ElectrosenseContext::getInstance()->getComprLevel(),
      ElectrosenseContext::getInstance()->getTcpHosts().c_str(),
      ElectrosenseContext::getInstance()->getTlsHosts().c_str(),
      ElectrosenseContext::getInstance()->getIface().c_str(),
      ElectrosenseContext::getInstance()->isFifoPriority() ? "true" : "false",
      ElectrosenseContext::getInstance()->getLog2FftSize(),
      ElectrosenseContext::getInstance()->getFFTbatchlen(),
      ElectrosenseContext::getInstance()->getAvgFactor(),
      ElectrosenseContext::getInstance()->getSoverlap());

  exit(-1);
}

void parse_args(int argc, char *argv[]) {

  int opt;
  const char *options = "hd:z:c:k:g:y:s:f:b:a:o:x:q:t:r:w:l:m:n:u:p:i:e:j:v:";

  // Option arguments
  while ( (opt = getopt(argc, argv, options)) != -1) {

    switch (opt) {
    case 'h':
      usage(argv[0]);
      break;
    case 'd':
      ElectrosenseContext::getInstance()->setDevIndex(atoi(optarg));
      break;
    case 'c':
      ElectrosenseContext::getInstance()->setClkOff(atoi(optarg));
      break;
    case 'z':
      ElectrosenseContext::getInstance()->setPipeline(strdup(optarg));
      break;
    case 'k':
      ElectrosenseContext::getInstance()->setClkCorrPerior(atoi(optarg));
      break;
    case 'g':
      if (atof(optarg) < -1) {
        std::cout << "Error usage: Gain can not be negative!\n" << std::endl;
        usage(argv[0]);
        break;
      }
      ElectrosenseContext::getInstance()->setGain(atof(optarg));
      break;
    case 'e':
      ElectrosenseContext::getInstance()->setBiasTee(atoi(optarg));
      break;
    case 'y':
      ElectrosenseContext::getInstance()->setHoppingStrategy(strdup(optarg));
      break;
    case 's':
      ElectrosenseContext::getInstance()->setSamplingRate(atol(optarg));
      break;
    case 'f':
      ElectrosenseContext::getInstance()->setLog2FftSize(atol(optarg));
      break;
    case 'b':
      ElectrosenseContext::getInstance()->setFFTbatchlen(atol(optarg));
      break;
    case 'a':
      ElectrosenseContext::getInstance()->setAvgFactor(atol(optarg));
      break;
    case 'o':
      ElectrosenseContext::getInstance()->setSoverlap(atol(optarg));
      break;
    case 'q':
      ElectrosenseContext::getInstance()->setFreqOverlap(atof(optarg));
      break;
    case 't':
      ElectrosenseContext::getInstance()->setMonitorTime(atol(optarg));
      break;
    case 'r':
      ElectrosenseContext::getInstance()->setMinTimeRes(atol(optarg));
      break;
    case 'w':
      ElectrosenseContext::getInstance()->setWindowing(strdup(optarg));
      break;
    case 'l':
      ElectrosenseContext::getInstance()->setComprLevel(atol(optarg));
      break;
    case 'm':
      ElectrosenseContext::getInstance()->setTcpHosts(strdup(optarg));
      break;
    case 'n':
      ElectrosenseContext::getInstance()->setTlsHosts(strdup(optarg));
      break;
    case 'p':
      ElectrosenseContext::getInstance()->setFifoPriority(1);
      break;
    case 'u':
      ElectrosenseContext::getInstance()->setOutputFileName(strdup(optarg));
      break;
    case 'x':
      ElectrosenseContext::getInstance()->setStartTimeSampling(atol(optarg));
      break;
    case 'i':
      ElectrosenseContext::getInstance()->setIface(strdup(optarg));
      break;
    //----------------------------------------------------------------------------------------------------------
    //  SSDF Attack additions 
    //----------------------------------------------------------------------------------------------------------
    case 'j':
      ElectrosenseContext::getInstance()->setBandwidth(atoll(optarg));
      break;
    case 'v':
      ElectrosenseContext::getInstance()->setMode(strdup(optarg));
      break;
    //----------------------------------------------------------------------------------------------------------
    default:
      usage(argv[0]);
    }
  }
  // Non-option arguments
  if (!((optind + 4 == argc) || (optind + 2 == argc))) {
	usage(argv[0]);

  } else if (optind+4==argc) {
    ElectrosenseContext::getInstance()->setMinFreq(atoll(argv[optind]));
    ElectrosenseContext::getInstance()->setMaxFreq(atoll(argv[optind + 1]));
    //----------------------------------------------------------------------------------------------------------
    //  SSDF Attack additions 
    //----------------------------------------------------------------------------------------------------------
    ElectrosenseContext::getInstance()->setAttackFreq1(atoll(argv[optind + 2]));
    ElectrosenseContext::getInstance()->setAttackFreq2(atoll(argv[optind + 3]));
    //----------------------------------------------------------------------------------------------------------
  }else {
    ElectrosenseContext::getInstance()->setMinFreq(atoll(argv[optind]));
    ElectrosenseContext::getInstance()->setMaxFreq(atoll(argv[optind + 1]));
    ElectrosenseContext::getInstance()->setAttackFreq1(1000000);
    ElectrosenseContext::getInstance()->setAttackFreq2(1200000);
  }

  
}

std::vector<Component *> vComponents;

void finish_ok() {

  std::cout << std::endl << "Shutdown components ..." << std::endl;
  for (unsigned int i = 0; i < vComponents.size(); i++) {
    std::cout << "  - Stopping component: " << vComponents.at(i)->getNameId() << std::endl;
    vComponents.at(i)->stop();
  }

  std::cout << "Sensing process finished correctly." << std::endl;
}

void signal_callback_handler(int signum) {

  std::cout << "signal_callback_handler " << signum << std::endl;
  finish_ok();
  exit(signum);
}

int main(int argc, char *argv[]) {

  signal(SIGINT, signal_callback_handler);
  signal(SIGTERM, signal_callback_handler);


  std::cout << std::endl
            << "Electrosense sensing application " << getElectrosenseVersion()
            << " (" << getElectrosenseTimeCompilation() << ")" << std::endl
            << std::endl;

  parse_args(argc, argv);
  ElectrosenseContext::getInstance()->print();

  electrosense::RemoveDC *rdcBlock;
  electrosense::RemoveDCRTL *rdcRTLBlock;
  electrosense::Windowing *winBlock;
  electrosense::FFT *fftBlock;
  electrosense::Averaging *avgBlock;
  electrosense::PSDFast *psdfastBlock;


  // A few checks before start
  if (ElectrosenseContext::getInstance()->getPipeline().compare("IQ") == 0) {
      if (ElectrosenseContext::getInstance()->getMinFreq() != ElectrosenseContext::getInstance()->getMaxFreq()) {
          std::cerr << "[ERROR] Min and Max frequencies in IQ mode must be the same!" << std::endl;
          exit(-1);
      }
  }

    // A few checks before start
    if (ElectrosenseContext::getInstance()->getPipeline().compare("PSD") == 0) {
        if (ElectrosenseContext::getInstance()->getMinFreq() >= ElectrosenseContext::getInstance()->getMaxFreq()) {
            std::cerr << "[ERROR] Min frequency must be lower than Max frequency!" << std::endl;
            exit(-1);
        }
    }

  // RTL-SDR Driver
  auto *rtlDriver = new electrosense::rtlsdrDriver();
  vComponents.push_back(rtlDriver);

  rtlDriver->open("0");

  electrosense::IQStream *iqStream;

  if (ElectrosenseContext::getInstance()->getPipeline().compare("PSD") == 0) {

    // RemoveDC Block
    rdcBlock = new electrosense::RemoveDC();
    rdcBlock->setQueueIn(rtlDriver->getQueueOut());
    vComponents.push_back(rdcBlock);

    // Windowing
    winBlock = new electrosense::Windowing(electrosense::Windowing::HAMMING);
    winBlock->setQueueIn(rdcBlock->getQueueOut());
    vComponents.push_back(winBlock);

    // FFT
    fftBlock = new electrosense::FFT();
    fftBlock->setQueueIn(winBlock->getQueueOut());
    vComponents.push_back(fftBlock);

    // Averaging

    avgBlock = new electrosense::Averaging();
    avgBlock->setQueueIn(fftBlock->getQueueOut());
    vComponents.push_back(avgBlock);

  } else if (ElectrosenseContext::getInstance()->getPipeline().compare("IQ") == 0) {

      // Comment the following to be able to run in RPi0
      /*rdcRTLBlock = new electrosense::RemoveDCRTL();
      vComponents.push_back(rdcRTLBlock);
      rdcRTLBlock->setQueueIn(rtlDriver->getQueueOut());
      */
  } else if (ElectrosenseContext::getInstance()->getPipeline().compare("DEC") == 0) {

    // PSD Fast block
    psdfastBlock = new electrosense::PSDFast();

    vComponents.push_back(psdfastBlock);
    psdfastBlock->setQueueIn(rtlDriver->getQueueOut2());

    // IQStream
    iqStream = new electrosense::IQStream();
    vComponents.push_back(iqStream);
    iqStream->setQueueIn(rtlDriver->getQueueOut());
  }

  electrosense::Transmission* transBlock;

  // Send measurements to the server.
  if (ElectrosenseContext::getInstance()->getTlsHosts().compare(DEFAULT_TLS_HOSTS) != 0) {

    auto *avroBlock = new electrosense::AvroSerialization();

    if (ElectrosenseContext::getInstance()->getPipeline().compare("PSD") == 0) {

      avroBlock->setQueueIn(avgBlock->getQueueOut());
      vComponents.push_back(avroBlock);

    } else if (ElectrosenseContext::getInstance()->getPipeline().compare("IQ") == 0) {

      rdcRTLBlock = new electrosense::RemoveDCRTL();
      rdcRTLBlock->setQueueIn(rtlDriver->getQueueOut());
      vComponents.push_back(rdcRTLBlock);

      avroBlock->setQueueIn(rdcRTLBlock->getQueueOut());
      vComponents.push_back(avroBlock);

    }

    transBlock = new electrosense::Transmission();
    vComponents.push_back(transBlock);
    transBlock->setQueueIn(avroBlock->getQueueOut());
  }

  // Send measurements to a  file.
  else if (ElectrosenseContext::getInstance()->getOutputFileName().compare(
               DEFAULT_OUTPUT_FILENAME) != 0) {

    if (ElectrosenseContext::getInstance()->getPipeline().compare("PSD") == 0) {

        auto* fileSink = new
        electrosense::FileSink(ElectrosenseContext::getInstance()->getOutputFileName());
        vComponents.push_back(fileSink);

        fileSink->setQueueIn(avgBlock->getQueueOut());

    } else if (ElectrosenseContext::getInstance()->getPipeline().compare("IQ") == 0) {

      if (ElectrosenseContext::getInstance()->getOutputType()==OUTPUT_TYPE_BYTE)
      {
          auto *iqSink = new electrosense::IQSink(ElectrosenseContext::getInstance()->getOutputFileName());
          iqSink->setQueueIn(rtlDriver->getQueueOut());
          vComponents.push_back(iqSink);
      }
      else { // FLOAT

          rdcRTLBlock = new electrosense::RemoveDCRTL();
          rdcRTLBlock->setQueueIn(rtlDriver->getQueueOut());
          vComponents.push_back(rdcRTLBlock);

          auto *iqSink = new electrosense::IQSink(ElectrosenseContext::getInstance()->getOutputFileName());
          iqSink->setQueueIn(rdcRTLBlock->getQueueOut());
          vComponents.push_back(iqSink);
      }

    }
  }


  try {

      std::cout << std::endl << "Starting components ..." << std::endl;
      for (unsigned int i = 0; i < vComponents.size(); i++) {
          std::cout << "  - Starting component: " << vComponents.at(i)->getNameId() << std::endl;
          vComponents.at(i)->start();
      }

      unsigned int current_dec = 0;

      while (1) {

          sleep(1);

          bool watch_components = true;
          for (unsigned int i = 0; i < vComponents.size(); i++) {
              if (!vComponents.at(i)->isRunning()) {
                  std::cout << "[WATCHER] " << vComponents.at(i)->getNameId() << " component is not running. Finishing process ..." << std::endl;
                  watch_components = false;
              }
          }
          if (!watch_components)
              break;

          if (ElectrosenseContext::getInstance()->getPipeline().compare("DEC") == 0) {

              // Restart IQStream and decoder if 1) decoder changes or 2) decoder
              // (child) dies
              if (ElectrosenseContext::getInstance()->getDecoder() != current_dec || (!iqStream->getDecoderState())) {

                  current_dec = ElectrosenseContext::getInstance()->getDecoder();

                  iqStream->stop();
                  usleep(0.03);

                  if (ElectrosenseContext::getInstance()->getDecoder() != -1)
                      iqStream->start();
              }
          }
      }
  }
  
  catch (...) {
        std::cerr << "Connection Server Lost" << std::endl;
  }

  finish_ok();

  return 0;
}
