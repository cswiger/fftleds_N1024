/*
 *  fftleds_N1024.ino - PCM capture and peak display example application
 *  Copyright 2018 Sony Semiconductor Solutions Corporation & Chuck Swiger
 *  With libraries from Adafruit https://github.com/adafruit/Adafruit_DotStar
 *  and Bernd Porr   https://github.com/berndporr/kiss-fft
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Audio.h>      // Sony Audio library
#include <Adafruit_DotStar.h>
#include <SPI.h>         // Hardware SPI for fast DotStars

#include "kiss_fftr.h"
#include "hammingwin.h"

#ifndef M_PI
#define M_PI 3.14159265358979324
#endif

#define N 1024       // size of fft 
#define NUMPIXELS 144 // Number of LEDs in strip
#define FFTSCALE 1024


AudioClass *theAudio;  
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

static const int32_t buffer_size = 1536;      // room for 768 samples, 2 bytes per, 1 ch mono, each fft of 1024 samples is 0.064 seconds at 16KHz for 15.625Hz
static char          s_buffer[buffer_size];
uint8_t   chunk=0;    // keep track of which chuck of samples we are on - basic unit is 256 samples (512bytes)
                       // chunk=0 1st 256, chunk 1 2nd 256 for 512, etc - this sketch is going for 512 samples (1024 bytes) N per fft
kiss_fft_scalar in[N];   // must be global for factoring into a seperate function from process

bool ErrEnd = false;

/**
 * @brief Audio attention callback
 *
 * When audio internal error occurc, this function will be called back.
 */

void audio_attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");
  
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
    {
      ErrEnd = true;
   }
}

/**
 *  @brief Setup audio device to capture PCM stream
 *
 *  Select input device as microphone <br>
 *  Set PCM capture sapling rate parameters to 16 kb/s <br>
 *  Set channel number 1 to capture audio from 1 microphone mono <br>
 *  System directory "/mnt/sd0/BIN" will be searched for PCM codec
 */
void setup()
{
  theAudio = AudioClass::getInstance();

  theAudio->begin(audio_attention_cb);

  // init the LED strip
  puts("init LED strip");
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  puts("initialization Audio Library");

  /* Select input device as microphone with max gain +21db for electret */
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC,210);

  /*
   * Set PCM capture sapling rate parameters to 16 kb/s. Set channel MONO
   * Search for PCM codec in "/mnt/sd0/BIN" directory
   */
  theAudio->initRecorder(AS_CODECTYPE_PCM, "/mnt/sd0/BIN", AS_SAMPLINGRATE_16000, AS_CHANNEL_MONO);
  puts("Init Recorder!");

  puts("Rec!");
  theAudio->startRecorder();
}

void doFFTOut(kiss_fft_scalar* in) {    // float in[N] must be global for this to work
  kiss_fftr_cfg cfg;
  kiss_fft_cpx out[N / 2 + 1];
  uint16_t idx;

  int threshold = analogRead(A0);   // varies from about 0 to 1000

  if ((cfg = kiss_fftr_alloc(N, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
  {
    kiss_fftr(cfg, in, out);
    free(cfg);
  }  else {
    printf("not enough memory?\n");
    exit(-1);
  }
  for(idx=20; idx<144+20; idx++) {         // for 1024 N-size fft this is quarter available real usable fft output, from 187.5 to 2156.25Hz 
    //if (abs(out[idx].r)+abs(out[idx].i) > threshold) {           // break pot threshold? 
    if (sqrt(out[idx].r * out[idx].r + out[idx].i * out[idx].i) > threshold) {           // break pot threshold? 
      long col = ((uint8_t)abs(out[idx].r))<<16 | ((uint8_t)abs(out[idx].i));       // real out is GREEN, imag is BLUE
      strip.setPixelColor(idx-20,col);
    } else {
      strip.setPixelColor(idx-20,0);
    }
  }
  strip.show();                           // update strip
}


/**
 * @brief Audio signal process for your application
 */


void signal_process(uint32_t size)    // this does not automatically get buffer_size each time! Usually 512 for 256 samples
{
  uint16_t idx;
  union Combine
  {
      short target;
      char dest[ sizeof( short ) ];
  };
  Combine cc;

  printf("%d %d\n",size, chunk);

  if (size==512 && chunk==0) {                         // got one chunk 
    chunk=1;        // go get the other parts
    for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[idx>>1] = (((double)cc.target)*hammingwin[idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
  }
  else if (size==512 && chunk==1) {
    chunk=2;
    for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[256+idx>>1] = (((double)cc.target)*hammingwin[256+idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
  } 
  else if (size==512 && chunk==2) {
    chunk=3;
    for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[512+idx>>1] = (((double)cc.target)*hammingwin[512+idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
  } 
  else if (size==512 && chunk==3) {
    chunk=0;
    for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[768+idx>>1] = (((double)cc.target)*hammingwin[768+idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
    doFFTOut(in); 
  } 

  else if (size==1024 && chunk==0) {                         // got one chunk 
    for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[idx>>1] = (((double)cc.target)*hammingwin[idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
    chunk=2;
  } 

  else if (size==1536 && chunk==0) {
      for(idx = 0; idx<size; idx+=2) {
      cc.dest[0] = s_buffer[idx];
      cc.dest[1] = s_buffer[idx+1];
      in[idx>>1] = (((double)cc.target)*hammingwin[idx>>1])/FFTSCALE;     // max scaling factor, prolly get by with less for the generally weak mic input
    }
    chunk=3;
  }
}

/**
 * @brief Execute frames for FIFO empty
 */

void execute_frames()
{
  uint32_t read_size = 0;
  do
    {
      err_t err = execute_aframe(&read_size);
      if ((err != AUDIOLIB_ECODE_OK)
       && (err != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA))
        {
          break;
        }
    }
  while (read_size > 0);
}

/**
 * @brief Execute one frame
 */

err_t execute_aframe(uint32_t* size)
{
  err_t err = theAudio->readFrames(s_buffer, buffer_size, size);

  if(((err == AUDIOLIB_ECODE_OK) || (err == AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA)) && (*size > 0)) 
    {
      int startT = micros();
      signal_process(*size);
      int stopT = micros();
      printf("signal_process time: %d\n",stopT-startT);
    }

  return err;
}

/**
 * @brief Capture frames of PCM data into buffer
 */
void loop() {

  static int32_t total_size = 0;
  uint32_t read_size =0;

  /* Execute audio data */
  err_t err = execute_aframe(&read_size);
  if (err != AUDIOLIB_ECODE_OK && err != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA)
    {
      theAudio->stopRecorder();
      goto exitRecording;
    }
  else if (read_size>0)
    {
      total_size += read_size;
    }


  /* Never Stop Recording */

  if (ErrEnd)
    {
      printf("Error End\n");
      theAudio->stopRecorder();
      goto exitRecording;
    }

  return;

exitRecording:

  theAudio->setReadyMode();
  theAudio->end();
  
  puts("End Recording");
  exit(1);

}


