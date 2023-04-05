# SNR_WDRC_Earpiece
Developed for the Lentz Lab

## Running in Arduino / Teensyduino
Follow the instructions at https://github.com/Tympan/Docs/wiki/Getting-Started-with-Tympan-Rev-E `Install Software`


## Connecting via matlab
look up com port on computer.
```
i>s8 = serialport("com8", 9600)
> writeline(s8, "r") // start recording
.... play sounds
> writeline(s8, "s") // stop recording
```
