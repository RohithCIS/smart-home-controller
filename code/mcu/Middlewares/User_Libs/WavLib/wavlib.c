#include <wavlib.h>

/*Return 0 on success and -1 on failure*/
wavfile_header_t get_PCM16_stereo_header(int32_t SampleRate,
                                         int32_t FrameCount)
{

  wavfile_header_t wav_header;
  int32_t subchunk2_size;
  int32_t chunk_size;

  subchunk2_size = FrameCount * NUM_CHANNELS * BITS_PER_SAMPLE / 8;
  chunk_size = 4 + (8 + SUBCHUNK1SIZE) + (8 + subchunk2_size);

  wav_header.ChunkID[0] = 'R';
  wav_header.ChunkID[1] = 'I';
  wav_header.ChunkID[2] = 'F';
  wav_header.ChunkID[3] = 'F';

  wav_header.ChunkSize = chunk_size;

  wav_header.Format[0] = 'W';
  wav_header.Format[1] = 'A';
  wav_header.Format[2] = 'V';
  wav_header.Format[3] = 'E';

  wav_header.Subchunk1ID[0] = 'f';
  wav_header.Subchunk1ID[1] = 'm';
  wav_header.Subchunk1ID[2] = 't';
  wav_header.Subchunk1ID[3] = ' ';

  wav_header.Subchunk1Size = SUBCHUNK1SIZE;
  wav_header.AudioFormat = AUDIO_FORMAT;
  wav_header.NumChannels = NUM_CHANNELS;
  wav_header.SampleRate = SampleRate;
  wav_header.ByteRate = BYTE_RATE;
  wav_header.BlockAlign = BLOCK_ALIGN;
  wav_header.BitsPerSample = BITS_PER_SAMPLE;

  wav_header.Subchunk2ID[0] = 'd';
  wav_header.Subchunk2ID[1] = 'a';
  wav_header.Subchunk2ID[2] = 't';
  wav_header.Subchunk2ID[3] = 'a';
  wav_header.Subchunk2Size = subchunk2_size;

  return wav_header;
}

/*Generate two saw-tooth signals at two frequencies and amplitudes*/
PCM16_stereo_t get_dual_sawtooth_sample(double frequency1,
                                        double amplitude1,
                                        double frequency2,
                                        double amplitude2,
                                        int32_t SampleRate,
                                        int32_t FrameCount,
                                        int32_t k)
{
  PCM16_stereo_t sample;
  double SampleRate_d = (double)SampleRate;
  double SamplePeriod = 1.0 / SampleRate_d;

  double Period1, Period2;
  double phase1, phase2;
  double Slope1, Slope2;

  /*Check for the violation of the Nyquist limit*/
  // if ((frequency1 * 2 >= SampleRate_d) || (frequency2 * 2 >= SampleRate_d))
  // {
  //   ret = -1;
  //   goto error0;
  // }

  /*Compute the period*/
  Period1 = 1.0 / frequency1;
  Period2 = 1.0 / frequency2;

  /*Compute the slope*/
  Slope1 = amplitude1 / Period1;
  Slope2 = amplitude2 / Period2;

  phase1 = k * SamplePeriod;
  phase1 = (phase1 > Period1) ? (phase1 - Period1) : phase1;

  phase2 = k * SamplePeriod;
  phase2 = (phase2 > Period2) ? (phase2 - Period2) : phase2;

  sample.left = (int16_t)(phase1 * Slope1);
  sample.right = (int16_t)(phase2 * Slope2);

  return sample;
}
