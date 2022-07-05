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

#ifndef ES_SENSOR_FFT_H
#define ES_SENSOR_FFT_H

#include <algorithm>
#include <complex.h>
#include <liquid/liquid.h>
#include <unistd.h>
#include <vector>

#include "../context/ElectrosenseContext.h"
#include "../drivers/Communication.h"
#include "../drivers/Component.h"
#include "../types/SpectrumSegment.h"

namespace electrosense {

class FFT : public Component,
            public Communication<SpectrumSegment *, SpectrumSegment *> {

public:
  FFT();

  ~FFT(){};

  std::string getNameId() { return std::string("FFT"); };

  int stop();

  ReaderWriterQueue<SpectrumSegment *> *getQueueIn() { return mQueueIn; }
  void setQueueIn(ReaderWriterQueue<SpectrumSegment *> *QueueIn) {
    mQueueIn = QueueIn;
  };

  ReaderWriterQueue<SpectrumSegment *> *getQueueOut() { return mQueueOut; };
  void setQueueOut(ReaderWriterQueue<SpectrumSegment *> *QueueOut){};

private:
  void run();

  void ComputeFFT_normal(std::vector<SpectrumSegment *> &segments);
  void ComputeFFT_repeat(std::vector<SpectrumSegment *> &segments, std::vector<std::complex<float>> &repeat_source_segment);
  void ComputeFFT_mimic(std::vector<SpectrumSegment *> &segments, int &current_frequency,  std::vector<std::complex<float>> &mimic_source_segment);
  void ComputeFFT_confusion(std::vector<SpectrumSegment *> &segments, int &current_frequency, std::vector<std::complex<float>> &confusion_source_segment_1, std::vector<std::complex<float>> &confusion_source_segment_2);
  void ComputeFFT_noise(std::vector<SpectrumSegment *> &segments);
  void ComputeFFT_spoof(std::vector<SpectrumSegment *> &segments, int &current_frequency, std::vector<std::complex<float>> &spoof_source_segment);
  void ComputeFFT_freeze(std::vector<SpectrumSegment *> &segments, int &current_frequency, bool &freeze, std::vector<std::complex<float>>  &freeze_source_segment);
  void ComputeFFT_delay(std::vector<SpectrumSegment *> &segments, int &current_frequency, int &affected_frequencies, int &current_iteration, int &delay, bool &init, bool &full, std::vector<std::vector<std::vector<std::complex<float>>>> &delay_source_segment, std::vector<std::complex<float>> &tmp_signal_vector);

  ReaderWriterQueue<SpectrumSegment *> *mQueueOut;
  ReaderWriterQueue<SpectrumSegment *> *mQueueIn;

  std::vector<SpectrumSegment *> mFFTBatch;
};

} // namespace electrosense

#endif // ES_SENSOR_FFT_H
