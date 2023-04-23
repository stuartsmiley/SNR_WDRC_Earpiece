//
// Based on work created by Eric Yuan and Chip Audette.
// 
// NEXT - 2 channel recording instead of 4?
//
//
// Purpose: Combine features from various examples to: 
// 1. Writing both audio and log at the same time.
// 2. Log compPerBand[Ichan].getCurrentGain_dB() at the sample frequency.
// 3. Use the typmpan ear pieces.
// 4. Use a common hearing aid algorithm for compression (see WDRC_FIR_8Band expample, but think stereo).
// 5. Potentiometer is out of picture. Volume level controlled programatically.
//
// You control this sketch through the USB Serial via the Arduino IDE's Serial Monitor.  You can always type an
// "h" (without quotes) to get the help menu.
//
//
// Hardware: 
//    Assumes that you're using a Tympan RevE with an Earpiece Shield
//    and two Tympan earpieces (each with a front and back PDM micrphone) 
//    connected through the earpiece audio ports (which uses the USB-B Mini connector).
//
// Mixing:
//    TODO: is this mix OK?
//    The front and back mic for each earpiece will be mixed into a single channel.
//    The output will be routed to both the Tympan AIC (i2s_out[0,1]) and the 
//    Shield AIC (i2s_out[2,3]), which can be heard using the earpiece receivers 
//    or a headphone plugged into the 3.5mm audio jacks on either the Tympan or Shield
//
//    This example does NOT use the EarpieceMixer class.  The EarpieceMixer class helps
//    manage all the potential left-right / front-back mixing that you might want to do,
//    but it also hides how one interacts with the hardware.  This sketch exposes some
//    of those details.  So, this sketch might be better to learn from, if you care to
//    learn the details.
//
// There is no support for the Tympan Remote App in this sketch
//
// MIT License.  Use at your own risk.  Have fun!
//

// here are the libraries that we need
#include <Tympan_Library.h>  //include the Tympan Library
#include <SD.h>

//set the sample rate and block size
const float sample_rate_Hz = 24000.0f ;  //24000 to 44117 to 96000 (or other frequencies in the table in AudioOutputI2S_F32)
const int audio_block_samples = 128;     //do not make bigger than audio_block_SAMPLES from AudioStream.h (which is 128)  Must be 128 for SD recording.

AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

const int N_CHAN = 10;

// define classes to control the Tympan and the AIC_Shield
Tympan           myTympan(TympanRev::E, audio_settings);         //choose TympanRev::D or TympanRev::E
EarpieceShield   earpieceShield(TympanRev::E, AICShieldRev::A);  //Note that EarpieceShield is defined in the Tympan_Libarary in AICShield.h 
SDClass sdx;

FsFile logFile;
boolean logFileOpen = false;
int experimentCount = 0;
// Instantiate the audio classes
AudioInputI2SQuad_F32   i2s_in(audio_settings);         //Bring audio in
AudioMixer4_F32         inputMixerL(audio_settings);    //For mixing (or not) the two mics in the left earpiece
AudioMixer4_F32         inputMixerR(audio_settings);    //For mixing (or not) the two mics in the right earpiece

AudioOutputI2SQuad_F32  i2s_out(audio_settings);        //Send audio out
AudioSDWriter_F32       audioSDWriter(&(sdx.sdfs), audio_settings);  //Write audio to the SD card (if activated)

AudioEffectGain_F32      preGain;
//here are the filters to break up the audio into multiple bands
AudioFilterFIR_F32       firFiltL[N_CHAN];
AudioFilterFIR_F32       firFiltR[N_CHAN];

//here are the per-band compressors
AudioEffectCompWDRC_F32  compPerBandL[N_CHAN];
AudioEffectCompWDRC_F32  compPerBandR[N_CHAN];

//here are the broad band compressors
AudioEffectCompWDRC_F32  compBroadband[2];

//mixer to reconstruct the broadband audio  left
AudioMixer8_F32          mixerL;
//mixer to reconstruct the broadband audio  right
AudioMixer8_F32          mixerR;

//Connect the front and rear mics (from each earpiece) to input mixer for the left ear
AudioConnection_F32     patchcord1(i2s_in, EarpieceShield::PDM_LEFT_FRONT,  inputMixerL, 0);    //Left-Front Mic
AudioConnection_F32     patchcord2(i2s_in, EarpieceShield::PDM_LEFT_REAR,   inputMixerL, 1);    //Left-Rear Mic
AudioConnection_F32     patchcord3(i2s_in, EarpieceShield::PDM_RIGHT_FRONT, inputMixerL, 2);    //Right-Front Mic
AudioConnection_F32     patchcord4(i2s_in, EarpieceShield::PDM_RIGHT_REAR,  inputMixerL, 3);    //Right-Rear Mic

//Connect the front and rear mics (from each earpiece) to input mixer for the right ear
AudioConnection_F32     patchcord5(i2s_in, EarpieceShield::PDM_LEFT_FRONT,  inputMixerR, 0);    //Left-Front Mic
AudioConnection_F32     patchcord6(i2s_in, EarpieceShield::PDM_LEFT_REAR,   inputMixerR, 1);    //Left-Rear Mic
AudioConnection_F32     patchcord7(i2s_in, EarpieceShield::PDM_RIGHT_FRONT, inputMixerR, 2);    //Right-Front Mic
AudioConnection_F32     patchcord8(i2s_in, EarpieceShield::PDM_RIGHT_REAR,  inputMixerR, 3);    //Right-Rear Mic

//connect to each of the filters to make the sub-bands
AudioConnection_F32     patchCord31(inputMixerL, 0, firFiltL[0], 0);
AudioConnection_F32     patchCord32(inputMixerL, 0, firFiltL[1], 0);
AudioConnection_F32     patchCord33(inputMixerL, 0, firFiltL[2], 0);
AudioConnection_F32     patchCord34(inputMixerL, 0, firFiltL[3], 0);
AudioConnection_F32     patchCord35(inputMixerL, 0, firFiltL[4], 0);
AudioConnection_F32     patchCord36(inputMixerL, 0, firFiltL[5], 0);
AudioConnection_F32     patchCord37(inputMixerL, 0, firFiltL[6], 0);
AudioConnection_F32     patchCord38(inputMixerL, 0, firFiltL[7], 0);
AudioConnection_F32     patchCord39(inputMixerL, 0, firFiltL[8], 0);
AudioConnection_F32     patchCord40(inputMixerL, 0, firFiltL[9], 0);


AudioConnection_F32     patchCord41(inputMixerR, 0, firFiltR[0], 0);
AudioConnection_F32     patchCord42(inputMixerR, 0, firFiltR[1], 0);
AudioConnection_F32     patchCord43(inputMixerR, 0, firFiltR[2], 0);
AudioConnection_F32     patchCord44(inputMixerR, 0, firFiltR[3], 0);
AudioConnection_F32     patchCord45(inputMixerR, 0, firFiltR[4], 0);
AudioConnection_F32     patchCord46(inputMixerR, 0, firFiltR[5], 0);
AudioConnection_F32     patchCord47(inputMixerR, 0, firFiltR[6], 0);
AudioConnection_F32     patchCord48(inputMixerR, 0, firFiltR[7], 0);
AudioConnection_F32     patchCord49(inputMixerR, 0, firFiltR[8], 0);
AudioConnection_F32     patchCord50(inputMixerR, 0, firFiltR[9], 0);


//connect each filter to its corresponding per-band compressor
AudioConnection_F32     patchCord51(firFiltL[0], 0, compPerBandL[0], 0);
AudioConnection_F32     patchCord52(firFiltL[1], 0, compPerBandL[1], 0);
AudioConnection_F32     patchCord53(firFiltL[2], 0, compPerBandL[2], 0);
AudioConnection_F32     patchCord54(firFiltL[3], 0, compPerBandL[3], 0);
AudioConnection_F32     patchCord55(firFiltL[4], 0, compPerBandL[4], 0);
AudioConnection_F32     patchCord56(firFiltL[5], 0, compPerBandL[5], 0);
AudioConnection_F32     patchCord57(firFiltL[6], 0, compPerBandL[6], 0);
AudioConnection_F32     patchCord58(firFiltL[7], 0, compPerBandL[7], 0);
AudioConnection_F32     patchCord59(firFiltL[8], 0, compPerBandL[8], 0);
AudioConnection_F32     patchCord60(firFiltL[9], 0, compPerBandL[9], 0);



AudioConnection_F32     patchCord61(firFiltR[0], 0, compPerBandR[0], 0);
AudioConnection_F32     patchCord62(firFiltR[1], 0, compPerBandR[1], 0);
AudioConnection_F32     patchCord63(firFiltR[2], 0, compPerBandR[2], 0);
AudioConnection_F32     patchCord64(firFiltR[3], 0, compPerBandR[3], 0);
AudioConnection_F32     patchCord65(firFiltR[4], 0, compPerBandR[4], 0);
AudioConnection_F32     patchCord66(firFiltR[5], 0, compPerBandR[5], 0);
AudioConnection_F32     patchCord67(firFiltR[6], 0, compPerBandR[6], 0);
AudioConnection_F32     patchCord68(firFiltR[7], 0, compPerBandR[7], 0);
AudioConnection_F32     patchCord69(firFiltR[8], 0, compPerBandR[8], 0);
AudioConnection_F32     patchCord70(firFiltR[9], 0, compPerBandR[9], 0);



//compute the output of the per-band compressors to the mixers (to make into one signal again)
AudioConnection_F32     patchCord71(compPerBandL[0], 0, mixerL, 0);
AudioConnection_F32     patchCord72(compPerBandL[1], 0, mixerL, 1);
AudioConnection_F32     patchCord73(compPerBandL[2], 0, mixerL, 2);
AudioConnection_F32     patchCord74(compPerBandL[3], 0, mixerL, 3);
AudioConnection_F32     patchCord75(compPerBandL[4], 0, mixerL, 4);
AudioConnection_F32     patchCord76(compPerBandL[5], 0, mixerL, 5);
AudioConnection_F32     patchCord77(compPerBandL[6], 0, mixerL, 6);
AudioConnection_F32     patchCord78(compPerBandL[7], 0, mixerL, 7);
AudioConnection_F32     patchCord79(compPerBandL[8], 0, mixerL, 8);
AudioConnection_F32     patchCord80(compPerBandL[9], 0, mixerL, 9);


AudioConnection_F32     patchCord81(compPerBandR[0], 0, mixerR, 0);
AudioConnection_F32     patchCord82(compPerBandR[1], 0, mixerR, 1);
AudioConnection_F32     patchCord83(compPerBandR[2], 0, mixerR, 2);
AudioConnection_F32     patchCord84(compPerBandR[3], 0, mixerR, 3);
AudioConnection_F32     patchCord85(compPerBandR[4], 0, mixerR, 4);
AudioConnection_F32     patchCord86(compPerBandR[5], 0, mixerR, 5);
AudioConnection_F32     patchCord87(compPerBandR[6], 0, mixerR, 6);
AudioConnection_F32     patchCord88(compPerBandR[7], 0, mixerR, 7);
AudioConnection_F32     patchCord89(compPerBandR[8], 0, mixerR, 8);
AudioConnection_F32     patchCord90(compPerBandR[9], 0, mixerR, 9);


//connect the output of the mixers to the final broadband compressor
AudioConnection_F32     patchCord91(mixerL, 0, compBroadband[0], 0);
AudioConnection_F32     patchCord92(mixerR, 0, compBroadband[1], 0);


//Connect the input mixers to both the Tympan and Shield audio outputs...which i2s output is associated with each audio output is in EarpieceShield.cpp  
AudioConnection_F32     patchcord11(inputMixerL, 0, i2s_out, EarpieceShield::OUTPUT_LEFT_TYMPAN);    //Tympan AIC, left output
AudioConnection_F32     patchcord12(inputMixerR, 0, i2s_out, EarpieceShield::OUTPUT_RIGHT_TYMPAN);   //Tympan AIC, right output
AudioConnection_F32     patchcord13(mixerL, 0, i2s_out, EarpieceShield::OUTPUT_LEFT_EARPIECE);  //Shield AIC, left output
AudioConnection_F32     patchcord14(mixerR, 0, i2s_out, EarpieceShield::OUTPUT_RIGHT_EARPIECE); //Shield AIC, right output

//Connect the input mixer to the SD card
AudioConnection_F32     patchcord21(i2s_in, EarpieceShield::PDM_LEFT_FRONT,  audioSDWriter, 0);   //connect Raw audio to SD writer
AudioConnection_F32     patchcord22(i2s_in, EarpieceShield::PDM_LEFT_REAR,   audioSDWriter, 1);   //connect Raw audio to SD writer
AudioConnection_F32     patchcord23(i2s_in, EarpieceShield::PDM_RIGHT_FRONT, audioSDWriter, 2);   //connect Raw audio to SD writer
AudioConnection_F32     patchcord24(i2s_in, EarpieceShield::PDM_RIGHT_REAR,  audioSDWriter, 3);   //connect Raw audio to SD writer

// //////////////// Manually define our own functions for choosing inputs and doing the front-back mixing
#include "MixerFunctions.h"  // for setInputSource() and for setInputMixer()


// //////////////// Control display and serial interaction via USB Serial
#include "State.h"                          //For enums
#include "SerialManager.h"                  //For processing serial communication
SerialManager                 serialManager;

//Static Variables
static float outputVolume_dB = 0.0;
static float inputGain_dB = 0.0;


// ///////////////// Main setup() and loop() as required for all Arduino programs
void setup() {

  //setup DC-blocking highpass filter running in the ADC hardware itself
  float cutoff_Hz = 40.0;  //set the default cutoff frequency for the highpass filter
  myTympan.setHPFonADC(true,cutoff_Hz,audio_settings.sample_rate_Hz); //set to false to disble

  if (!sdx.sdfs.begin(SdioConfig(FIFO_SDIO))) {
    sdx.sdfs.errorHalt(&Serial, "setup: SD begin failed!");
  }
 
  myTympan.beginBothSerial(); delay(1500);
  Serial.println("EarpieceManualMixing_wSD: setup():...");
  Serial.print("Sample Rate (Hz): "); Serial.println(audio_settings.sample_rate_Hz);
  Serial.print("Audio Block Size (samples): "); Serial.println(audio_settings.audio_block_samples);

  //allocate the dynamic memory for audio processing blocks
  AudioMemory_F32(40,audio_settings); //I can only seem to allocate 400 blocks

  //Enable the Tympan and AIC shields to start the audio flowing!
  myTympan.enable(); 
  earpieceShield.enable();

  //Set the state of the LEDs
  myTympan.setRedLED(HIGH);
  myTympan.setAmberLED(LOW);

  //prepare the SD writer for the format that we want and any error statements
  audioSDWriter.setSerial(&myTympan);
  audioSDWriter.setNumWriteChannels(4);             //four channels for this quad recorder, but you could set it to 2
  Serial.print("SD configured for "); Serial.print(audioSDWriter.getNumWriteChannels()); Serial.println(" channels.");
  setupAudioProcessing();

  //set headphone volume (Potentiometer disabled. -5.0 is a comfortable level for normal hearing)
  setOutputVolume_dB(-5.0); //dB, -63.6 to +24 dB in 0.5dB steps.

  //Choose the input source
  setInputSource(INPUT_PDMMICS); //choose PDM Mics, which are the mics in the Tympan Earpieces

  //For each earpiece, mix front and back mics equally
  setInputMixer(ALL_MICS, 0.5);
  
    //set volumes
  setOutputGain_dB(0.f);  // -63.6 to +24 dB in 0.5dB steps.  uses signed 8-bit
  float default_mic_input_gain_dB = 15.0f; //gain on the microphone
  setInputGain_dB(default_mic_input_gain_dB); // set MICPGA volume, 0-47.5dB in 0.5dB setps

 
  //End of setup
  Serial.println("Setup: complete."); 
  serialManager.printHelp();

}

static void configureBroadbandWDRCs(float fs_Hz, BTNRH_WDRC::CHA_WDRC *gha, AudioEffectCompWDRC_F32 *WDRC) {
  //logic and values are extracted from from CHAPRO repo agc_prepare.c...the part setting CHA_DVAR

  //extract the parameters
  float atk = (float)gha->attack;  //milliseconds!
  float rel = (float)gha->release; //milliseconds!
  //float fs = gha->fs;
  float fs = (float)fs_Hz; // WEA override...not taken from gha
  float maxdB = (float) gha->maxdB;
  float exp_cr = (float)gha->exp_cr;
  float exp_end_knee = (float)gha->exp_end_knee;
  float tk = (float) gha->tk;
  float comp_ratio = (float) gha->cr;
  float tkgain = (float) gha->tkgain;
  float bolt = (float) gha->bolt;

  //set the compressor's parameters
  WDRC->setSampleRate_Hz(fs);
  WDRC->setParams(atk,rel,maxdB,exp_cr,exp_end_knee,tkgain,comp_ratio,tk,bolt);

}

static void configurePerBandWDRCs(int nchan, float fs_Hz, BTNRH_WDRC::CHA_DSL *dsl, BTNRH_WDRC::CHA_WDRC *gha, AudioEffectCompWDRC_F32 WDRCs[]) {

  if (nchan > dsl->nchannel) {
    myTympan.println(F("configureWDRC.configure: *** ERROR ***: nchan > dsl.nchannel"));
    myTympan.print(F("    : nchan = ")); myTympan.println(nchan);
    myTympan.print(F("    : dsl.nchannel = ")); myTympan.println(dsl->nchannel);
  }

  //now, loop over each channel
  for (int i=0; i < nchan; i++) {

    //logic and values are extracted from from CHAPRO repo agc_prepare.c
    float atk = (float)dsl->attack;   //milliseconds!
    float rel = (float)dsl->release;  //milliseconds!
    float fs = (float) fs_Hz; // WEA override
    float maxdB = (float) dsl->maxdB;
    float exp_cr = (float) dsl->exp_cr[i];
    float exp_end_knee = (float) dsl->exp_end_knee[i];
    float tk = (float) dsl->tk[i];
    float comp_ratio = (float) dsl->cr[i];
    float tkgain = (float) dsl->tkgain[i];
    float bolt = (float) dsl->bolt[i];

    // adjust BOLT
    float cltk = (float)gha->tk;
    if (bolt > cltk) bolt = cltk;
    if (tkgain < 0) bolt = bolt + tkgain;

    //set the compressor's parameters
    WDRCs[i].setSampleRate_Hz(fs);
    WDRCs[i].setParams(atk,rel,maxdB,exp_cr,exp_end_knee,tkgain,comp_ratio,tk,bolt);
  }
}


//define functions to setup the audio processing parameters
#define N_FIR 96
float firCoeff[N_CHAN][N_FIR];
void setupAudioProcessing(void) {
  //set the pre-gain (if used)
  preGain.setGain_dB(0.0f);

  //set the per-channel filter coefficients
  //#include "GHA_Constants.h"  //this sets dsl and gha, which are used in the next line
  #include "FLAT_M_SLOW.h"
  AudioConfigFIRFilterBank_F32 makeFIRcoeffs(N_CHAN, N_FIR, audio_settings.sample_rate_Hz, (float *)dsl.cross_freq, (float *)firCoeff);
  for (int i=0; i< N_CHAN; i++) {
    firFiltL[i].begin(firCoeff[i], N_FIR, audio_settings.audio_block_samples);
    firFiltR[i].begin(firCoeff[i], N_FIR, audio_settings.audio_block_samples);
  }

  //setup all of the the compressors
  configureBroadbandWDRCs(audio_settings.sample_rate_Hz, &gha, &compBroadband[0]);
  configureBroadbandWDRCs(audio_settings.sample_rate_Hz, &gha, &compBroadband[1]);
  configurePerBandWDRCs(N_CHAN, audio_settings.sample_rate_Hz, &dsl, &gha, compPerBandL);
  configurePerBandWDRCs(N_CHAN, audio_settings.sample_rate_Hz, &dsl, &gha, compPerBandR);
}

void printCompressorState(unsigned long curTime_micros, unsigned long updatePeriod_micros) {
  static unsigned long lastUpdate_micros = 0;
  if (curTime_micros < lastUpdate_micros) {
    lastUpdate_micros = 0;
  }
  if ((curTime_micros - lastUpdate_micros) > updatePeriod_micros) {
    // TODO: would it be better to concat all output to a string and do a single logfile.println() ?
    logFile.print(curTime_micros);
    logFile.print(' ');
    for (int i = 0; i < N_CHAN;  i++) {
      logFile.print(compPerBandL[i].getCurrentGain_dB());
      logFile.print(" ");
    }
    for (int i = 0; i < N_CHAN;  i++) {
      logFile.print(compPerBandR[i].getCurrentGain_dB());
      if (i < (N_CHAN - 1)) {
          logFile.print(" ");
      }
    }
    logFile.println();
  }
}

void loop() {

  if (logFileOpen) {
    printCompressorState(micros(), 22);
    
  }
  //respond to Serial commands
  while (Serial.available()) serialManager.respondToByte((char)Serial.read());   //USB Serial
  //while (Serial1.available()) serialManager.respondToByte((char)Serial1.read()); //BT Serial

  //service the SD recording
  audioSDWriter.serviceSD_withWarnings(i2s_in); //For the warnings, it asks the i2s_in class for some info
  
  //service the LEDs...blink slow normally, blink fast if recording
  myTympan.serviceLEDs(millis(),audioSDWriter.getState() == AudioSDWriter::STATE::RECORDING); 

  // periodicallly check the potentiometer
  //servicePotentiometer(millis(),100); //service the potentiometer every 100 msec
}


// ///////////////// Servicing routines

//servicePotentiometer: listens to the blue potentiometer and sends the new pot value
//  to the audio processing algorithm as a control parameter
void servicePotentiometer(unsigned long curTime_millis, unsigned long updatePeriod_millis) {
  //static unsigned long updatePeriod_millis = 100; //how many milliseconds between updating the potentiometer reading?
  static unsigned long lastUpdate_millis = 0;
  static float prev_val = -1.0;

  //has enough time passed to update everything?
  if (curTime_millis < lastUpdate_millis) {
    lastUpdate_millis = 0; //handle wrap-around of the clock
  }
  
  if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) { //is it time to update the user interface?
    //read potentiometer
    float val = float(myTympan.readPotentiometer()) / 1023.0; //Output 0.0 to 1.0
    val = (1.0/8.0) * (float)((int)(8.0 * val + 0.5)); //quantize X steps (to reduce chatter).  Output 0.0 to 1.0

    //send the potentiometer value to your algorithm as a control parameter
    if (abs(val - prev_val) > 0.05) { //is it different than before?
      prev_val = val;  //save the value for comparison for the next time around

      //choose the desired gain value based on the knob setting
      const float min_gain_dB = -10.0, max_gain_dB = 30.0; //set desired gain range
      float vol_dB = min_gain_dB + (max_gain_dB - min_gain_dB)*val; //computed desired gain value in dB

      //command the new gain setting
      setOutputVolume_dB(vol_dB);
      Serial.print("servicePotentiometer: Headphone (dB) = "); Serial.println(vol_dB); //print text to Serial port for debugging
    }
    
    lastUpdate_millis = curTime_millis;
    
  } // end if
} //end servicePotentiometer();



// ////////////// Change settings of system from the Serial Monitor

float setOutputGain_dB(float gain_dB) {  return myTympan.volume_dB(gain_dB); }


//here's a function to change the volume settings.   We'll also invoke it from our serialManager
void incrementInputGain(float increment_dB) { setInputGain_dB(inputGain_dB+increment_dB);}
void setInputGain_dB(float newGain_dB) { 
  //Record new gain
  inputGain_dB = newGain_dB;

  //Set gain
  myTympan.setInputGain_dB(inputGain_dB);   //set the AIC on the main Tympan board
  earpieceShield.setInputGain_dB(inputGain_dB);  //set the AIC on the Earpiece Shield
  Serial.print("Input Gain: "); Serial.print(inputGain_dB); Serial.println("dB");
}
float setDigitalGain_dB(float gain_dB) { return setDigitalGain_dB(gain_dB, true); }
float setDigitalGain_dB(float gain_dB, bool printToUSBSerial) {
  // or should this be compBroadband[0] and compBroadband[1]
  float digital_gain = compBroadband[0].setGain_dB(gain_dB); //this actually sets the gain
  compBroadband[1].setGain_dB(gain_dB); //this actually sets the gain

  return digital_gain;
}
//Increment Headphone Output Volume
void incrementKnobGain(float increment_dB) {  setOutputVolume_dB(outputVolume_dB+increment_dB);}
void setOutputVolume_dB(float newVol_dB) {
  //Update output volume;Limit vol_dB to safe values
  outputVolume_dB = max(min(newVol_dB, 24.0),-60.0);

  //Set output volume
  myTympan.volume_dB(outputVolume_dB);                   // headphone amplifier.  -63.6 to +24 dB in 0.5dB steps.
  earpieceShield.volume_dB(outputVolume_dB);
  Serial.print("Output Volume: "); Serial.print(outputVolume_dB); Serial.println("dB");
}

void startLogging(char *fname) {
  //myTympan.println("START LOGGING NOW");
  logFile = sdx.sdfs.open(fname, O_WRITE | O_CREAT);
  logFileOpen = true;
  logFile.println("FLAT-M/SLOW");
}

void stopLogging() {
  logFileOpen = false;
  logFile.close();
  //myTympan.println("END LOGGING");
}

//void writeTextToSD(String myString) {
//
//  //adapted from Teensy SD example: SdFat_Usage.ino
//  Serial.println("writeTextToSD: opening " + String(log_filename));
//  FsFile myfile = sdx.sdfs.open(log_filename, O_WRITE | O_CREAT | O_APPEND);  //will append to the end of the file
//  Serial.println("writeTextToSD: writing: " + myString);
//  myfile.println(myString); //write the text myfile.close();
//  Serial.println("writeTextToSD: closing file.");
//  myfile.close(); 
//}

void startExperiment() {
  experimentCount++;
  char fname[] = "AUDIOxxx.WAV";
  char exp_fname[] = "EXPxxx.txt";
  int hundreds = experimentCount / 100;
  fname[5] = hundreds + '0';  //stupid way to convert the number to a character
  exp_fname[3] = hundreds + '0';
  int tens = (experimentCount - (hundreds*100)) / 10;  //truncates
  fname[6] = tens + '0';  //stupid way to convert the number to a character
  exp_fname[4] = tens + '0'; 
  int ones = experimentCount - (tens * 10) - (hundreds*100);
  fname[7] = ones + '0';  //stupid way to convert the number to a character
  exp_fname[5] = ones + '0';
  audioSDWriter.startRecording(fname);
  startLogging(exp_fname); 
}
