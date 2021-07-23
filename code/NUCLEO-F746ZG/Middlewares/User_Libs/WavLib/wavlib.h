#include "stdint.h"
#include "limits.h"

typedef struct wavfile_header_s
{
  char ChunkID[4];   /*  4   */
  int32_t ChunkSize; /*  4   */
  char Format[4];    /*  4   */

  char Subchunk1ID[4];   /*  4   */
  int32_t Subchunk1Size; /*  4   */
  int16_t AudioFormat;   /*  2   */
  int16_t NumChannels;   /*  2   */
  int32_t SampleRate;    /*  4   */
  int32_t ByteRate;      /*  4   */
  int16_t BlockAlign;    /*  2   */
  int16_t BitsPerSample; /*  2   */

  char Subchunk2ID[4];
  int32_t Subchunk2Size;
} wavfile_header_t;

/*Data structure to hold a single frame with two channels*/
typedef struct PCM16_stereo_t
{
  int16_t left;
  int16_t right;
} PCM16_stereo_t;

/*Standard values for CD-quality audio*/
#define SUBCHUNK1SIZE (16)
#define AUDIO_FORMAT (1) /*For PCM*/
#define NUM_CHANNELS (2)
#define SAMPLE_RATE (8000)

#define BITS_PER_SAMPLE (16)

#define BYTE_RATE (SAMPLE_RATE * NUM_CHANNELS * BITS_PER_SAMPLE / 8)
#define BLOCK_ALIGN (NUM_CHANNELS * BITS_PER_SAMPLE / 8)

wavfile_header_t get_PCM16_stereo_header(int32_t, int32_t);
PCM16_stereo_t get_dual_sawtooth_sample(double, double, double, double, int32_t, int32_t, int32_t);
